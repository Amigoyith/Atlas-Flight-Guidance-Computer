/**
 * @file atlas_stabilization.c
 * @brief Bench-only gravity direction estimator, calibration and bounded horn targets.
 * Major functions: Update, Calibrate, Targets, CommandsStep and CRC-checked SD records.
 * Coordinates: sensor +Z points out of the STM32 face. Calibrated upright defines
 * body +Z; body +X is that face projected normal to +Z. Azimuths are clockwise
 * looking down from USB-C: PCB 7=45, 8=135, 1=225, 2=315 degrees.
 */
#include "atlas_stabilization.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

_Static_assert(sizeof(AtlasStabilizationConfig) == ATLAS_STAB_CONFIG_BYTES, "SD record size");
_Static_assert(offsetof(AtlasStabilizationConfig, crc) == 60U, "SD record layout");
#define STAB_MAGIC UINT32_C(0x41535442)
#define RAD_PER_DEG 0.01745329251994329577f
/* Nominal LSM6DSV16B rate (240 Hz); the true period is set per update. Gain 2/s
 * is a 0.5 s accelerometer time constant. Acceleration beyond 10 degrees of
 * disagreement is ignored for up to 5 s, then the accelerometer is trusted again. */
#define STAB_AHRS_RATE_HZ 240.0f
#define STAB_AHRS_GAIN 2.0f
#define STAB_AHRS_REJECT_DEG 10.0f
#define STAB_AHRS_TIMEOUT_S 5.0f
#define STAB_AHRS_MAG_REJECT_DEG 20.0f /* Looser than tilt: hard-iron error is common. */

/** @brief Vector dot product. @param a First. @param b Second. @return Scalar. */
static float stab_dot(const float a[3], const float b[3])
{ return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]; }
/** @brief Vector norm. @param v Input. @return Magnitude. */
static float stab_norm(const float v[3]) { return sqrtf(stab_dot(v, v)); }
/** @brief In-place unit vector with finite checks. @param v Vector. @return Validity. */
static bool stab_unit(float v[3])
{
    const float n = stab_norm(v);
    if (!isfinite(n) || n < 0.001f) return false;
    for (unsigned i=0; i<3; ++i) v[i] /= n;
    return true;
}
/** @brief Right-handed cross product. @param a First. @param b Second. @param out Result. */
static void stab_cross(const float a[3], const float b[3], float out[3])
{
    out[0]=a[1]*b[2]-a[2]*b[1]; out[1]=a[2]*b[0]-a[0]*b[2]; out[2]=a[0]*b[1]-a[1]*b[0];
}
/** @brief CRC32/ISO-HDLC, no hardware dependency. @param c Record. @return Checksum. */
static uint32_t stab_crc(const AtlasStabilizationConfig *c)
{
    const uint8_t *bytes=(const uint8_t *)c;
    uint32_t crc=UINT32_MAX;
    for (unsigned i=0; i<60U; ++i)
    {
        crc ^= bytes[i];
        for (unsigned bit=0; bit<8U; ++bit)
            crc=(crc>>1) ^ ((crc&1U) ? UINT32_C(0xEDB88320) : 0U);
    }
    return ~crc;
}
/** @brief Seal an explicitly prepared record. @param c Record. */
void AtlasStabilization_Seal(AtlasStabilizationConfig *c)
{ c->magic=STAB_MAGIC; c->version=1U; memset(c->reserved,0,sizeof(c->reserved)); c->crc=stab_crc(c); }
/** @brief Validate supported settings and upright mounting. @param c Record. @return Validity. */
bool AtlasStabilization_ConfigValid(const AtlasStabilizationConfig *c)
{
    if (!c || c->magic!=STAB_MAGIC || c->version!=1U || c->enabled>1U || c->reverse_mask>15U ||
        c->crc!=stab_crc(c)) return false;
    for (unsigned i=0; i<5U; ++i) if(c->reserved[i]) return false;
    for (unsigned i=0; i<3U; ++i)
        if (!isfinite(c->upright[i]) || !isfinite(c->bias_dps[i]) || fabsf(c->bias_dps[i])>10.0f) return false;
    const float n=stab_norm(c->upright);
    return n>0.99f && n<1.01f && c->upright[1]<-0.85f && fabsf(c->upright[2])<0.35f;
}
/** @brief Publish the AHRS up direction (sensor axes, the direction the accelerometer reads at rest).
 * @param s State. @return False if the result is not finite, so the estimator can fault. */
static bool stab_ahrs_up(AtlasStabilization *s)
{
    const FusionVector g=FusionAhrsGetGravity(&s->ahrs);
    s->up[0]=g.axis.x; s->up[1]=g.axis.y; s->up[2]=g.axis.z;
    return stab_unit(s->up);
}
/** @brief Heading of ATLAS_STAB_HEADING_AXIS, clockwise from magnetic north (Fusion is ENU).
 * R maps sensor to earth, so column k holds that axis in (east, north, up).
 * @param s State. @return False when the axis is too close to vertical to define a heading. */
static bool stab_heading(AtlasStabilization *s)
{
    const FusionMatrix r=FusionQuaternionToMatrix(FusionAhrsGetQuaternion(&s->ahrs));
    const float east=r.array[ATLAS_STAB_HEADING_AXIS], north=r.array[3U+ATLAS_STAB_HEADING_AXIS];
    if(!isfinite(north) || !isfinite(east) || hypotf(north,east)<ATLAS_STAB_HEADING_MIN_HORIZONTAL) return false;
    float h=atan2f(east,north)/RAD_PER_DEG;
    if(h<0.0f) h+=360.0f;
    s->heading_deg=h>=360.0f?0.0f:h;
    return true;
}
/** @brief Latch Fusion's flags, rejection internals and heading for diagnostics. @param s State.
 * @param mag_used True if a magnetometer sample was fed to this step. */
static void stab_ahrs_flags(AtlasStabilization *s,bool mag_used)
{
    const FusionAhrsFlags f=FusionAhrsGetFlags(&s->ahrs);
    const FusionAhrsInternalStates i=FusionAhrsGetInternalStates(&s->ahrs);
    s->accel_error_deg=i.accelerationError; s->accel_trigger=i.accelerationRecoveryTrigger;
    s->mag_error_deg=i.magneticError; s->mag_trigger=i.magneticRecoveryTrigger;
    /* Lock once Fusion accepts the field and agrees with it; lose it if the field goes away before launch.
     * After launch the gyro coasts the heading, so the lock is kept (the LAUNCHED flag says so). */
    if(mag_used && !f.startup && !i.magnetometerIgnored && i.magneticError<ATLAS_STAB_MAG_LOCK_DEG) s->mag_locked=true;
    else if(!s->launched && (int32_t)(s->timestamp_ms-s->mag_ms)>(int32_t)(ATLAS_STAB_MAG_MAX_AGE_MS*4U)) s->mag_locked=false;
    const bool heading_ok=stab_heading(s);
    s->flags=(s->launched?ATLAS_STAB_FLAG_LAUNCHED:0U)|(f.startup?ATLAS_STAB_FLAG_STARTUP:0U)|
        (i.accelerometerIgnored?ATLAS_STAB_FLAG_ACCEL_IGNORED:0U)|
        (f.accelerationRecovery?ATLAS_STAB_FLAG_ACCEL_RECOVERY:0U)|
        (f.overrangeRecovery?ATLAS_STAB_FLAG_OVERRANGE_RECOVERY:0U)|
        ((!mag_used||i.magnetometerIgnored)?ATLAS_STAB_FLAG_MAG_IGNORED:0U)|
        (f.magneticRecovery?ATLAS_STAB_FLAG_MAG_RECOVERY:0U)|
        ((s->mag_locked&&heading_ok)?ATLAS_STAB_FLAG_HEADING_VALID:0U);
}
/** @brief Condition the optional magnetometer: freshness, finiteness, hard-iron offset, axis map, plausible
 * magnitude. @param s State. @param sample Input. @param m Output in IMU axes (gauss).
 * @return True if usable this step. */
static bool stab_mag(AtlasStabilization *s,const AtlasStabilizationSample *sample,float m[3])
{
    static const float map[9]=ATLAS_STAB_MAG_TO_IMU, offset[3]=ATLAS_STAB_MAG_OFFSET_GAUSS;
    if(!ATLAS_STAB_USE_MAGNETOMETER) return false;
    if(!s->mag_in_valid || (int32_t)(sample->timestamp_ms-s->mag_in_ms)>(int32_t)ATLAS_STAB_MAG_MAX_AGE_MS ||
       (int32_t)(sample->timestamp_ms-s->mag_in_ms)<-(int32_t)ATLAS_STAB_MAG_MAX_AGE_MS) return false;
    float raw[3];
    for(unsigned i=0;i<3U;++i)
    { raw[i]=s->mag_in[i]-offset[i]; if(!isfinite(raw[i])) return false; }
    for(unsigned r=0;r<3U;++r) m[r]=map[3U*r]*raw[0]+map[3U*r+1U]*raw[1]+map[3U*r+2U]*raw[2];
    const float n=stab_norm(m);
    if(!isfinite(n) || n<ATLAS_STAB_MAG_MIN_GAUSS || n>ATLAS_STAB_MAG_MAX_GAUSS) return false;
    s->mag_ms=s->mag_in_ms;
    return true;
}
/** @brief Advance the AHRS by one measured interval. @param s State. @param w Bias-corrected gyro (dps).
 * @param a Accelerometer (g) or NULL to ignore it. @param m Magnetometer (IMU axes) or NULL.
 * @param dt Elapsed seconds. */
static void stab_ahrs_step(AtlasStabilization *s,const float w[3],const float a[3],const float m[3],float dt)
{
    const FusionVector gyro={.array={w[0],w[1],w[2]}};
    const FusionVector accel=a?(FusionVector){.array={a[0],a[1],a[2]}}:FUSION_VECTOR_ZERO;
    const FusionVector mag=m?(FusionVector){.array={m[0],m[1],m[2]}}:FUSION_VECTOR_ZERO;
    FusionAhrsSetSamplePeriod(&s->ahrs,dt);
    FusionAhrsUpdate(&s->ahrs,gyro,accel,mag);
    stab_ahrs_flags(s,m!=NULL);
}
/** @brief Start the Fusion AHRS at the measured up direction, skipping its convergence ramp.
 * @param s State. @param a Accelerometer (g). @param an Its norm, already range checked.
 * @param have_mag True keeps Fusion's startup ramp, which ignores rejection while heading locks to the field. */
static void stab_ahrs_start(AtlasStabilization *s,const float a[3],float an,bool have_mag)
{
    FusionAhrsSettings settings=fusionAhrsDefaultSettings;
    settings.convention=FusionConventionEnu; /* Project-wide earth frame: East, North, Up. */
    settings.sampleRate=STAB_AHRS_RATE_HZ; settings.gain=STAB_AHRS_GAIN;
    settings.accelerationRejection=STAB_AHRS_REJECT_DEG; settings.magneticRejection=STAB_AHRS_MAG_REJECT_DEG; settings.rejectionTimeout=STAB_AHRS_TIMEOUT_S;
    FusionAhrsInitialise(&s->ahrs); FusionAhrsSetSettings(&s->ahrs,&settings);
    /* Third row of the attitude matrix is the up direction in sensor axes, so
     * q=(w,x,y,0) with w=sqrt((1+uz)/2), x=uy/2w, y=-ux/2w reproduces u exactly. */
    const float ux=a[0]/an,uy=a[1]/an,uz=a[2]/an;
    FusionQuaternion q={.element={0.0f,1.0f,0.0f,0.0f}}; /* 180 degrees about X: u=-Z. */
    if(uz>-0.999f)
    {
        const float w=sqrtf(0.5f*(1.0f+uz));
        q.element.w=w; q.element.x=uy/(2.0f*w); q.element.y=-ux/(2.0f*w); q.element.z=0.0f;
    }
    FusionAhrsSetQuaternion(&s->ahrs,FusionQuaternionNormalise(q));
    if(!have_mag) FusionAhrsSkipStartup(&s->ahrs);
    s->mag_locked=false; s->mag_ms=0U; s->heading_deg=0.0f;
    stab_ahrs_flags(s,false);
    (void)stab_ahrs_up(s);
}
/** @brief Store the newest magnetometer reading for the next Update; ignored after launch.
 * @param s Estimator. @param m Reading, or NULL to clear. */
void AtlasStabilization_SetMagnetometer(AtlasStabilization *s, const AtlasStabilizationMagnetometer *m)
{
    s->mag_in_valid=m && m->valid;
    if(!s->mag_in_valid) return;
    memcpy(s->mag_in,m->gauss,sizeof(s->mag_in)); s->mag_in_ms=m->timestamp_ms;
}
/** @brief Latch launch: attitude becomes gyro-only from the current estimate.
 * @param s Estimator. @return False if there is no attitude yet to coast from. */
bool AtlasStabilization_Launch(AtlasStabilization *s)
{
    if(!s->initialized) return false;
    s->launched=true; s->ready=true; s->fault=ATLAS_STAB_FAULT_NONE; s->still_count=0U;
    s->flags|=ATLAS_STAB_FLAG_LAUNCHED|ATLAS_STAB_FLAG_ACCEL_IGNORED|ATLAS_STAB_FLAG_MAG_IGNORED;
    return true;
}
/** @brief Fuse one new sample and enforce independent freshness. @param s State.
 * @param sample Sensor input. @param c Optional bias. @param now Current tick. */
void AtlasStabilization_Update(AtlasStabilization *s, const AtlasStabilizationSample *sample,
                               const AtlasStabilizationConfig *c, uint32_t now)
{
    if (s->launched && !sample->valid) { s->fault=ATLAS_STAB_FAULT_SAMPLE; return; } /* Coast; never restart. */
    if (!s->launched && (!sample->valid || (uint32_t)(now-sample->timestamp_ms)>ATLAS_STAB_SAMPLE_MAX_MS))
    {
        s->fault=sample->valid?ATLAS_STAB_FAULT_STALE:ATLAS_STAB_FAULT_SAMPLE;
        s->initialized=s->ready=false; s->still_count=0U; return;
    }
    const uint32_t elapsed=(uint32_t)(sample->timestamp_ms-s->timestamp_ms);
    if (s->initialized && elapsed==0U) return;
    float a[3],w[3];
    for(unsigned i=0;i<3U;++i)
    {
        a[i]=sample->accel_g[i];
        w[i]=sample->gyro_dps[i]-(c ? c->bias_dps[i] : 0.0f);
    }
    const float an=stab_norm(a), wn=stab_norm(w);
    /* Boost: a sustained high-g reading latches launch. From then on gravity is
     * unobservable, so attitude coasts on the gyro alone and no accelerometer, rate-limit
     * or sample-gap check may restart it from a (meaningless) acceleration vector. */
    if (s->initialized && !s->launched && isfinite(an) && isfinite(wn))
    {
        s->launch_count = (an>=ATLAS_STAB_LAUNCH_G) ? s->launch_count+1U : 0U;
        if (s->launch_count>=ATLAS_STAB_LAUNCH_SAMPLES) (void)AtlasStabilization_Launch(s);
    }
    /* Candidate boost samples coast too, so the estimator is not reset before the latch. */
    if (s->launched || (s->initialized && s->launch_count>0U))
    {
        if (!isfinite(wn)) { s->fault=ATLAS_STAB_FAULT_NONFINITE; return; }
        stab_ahrs_step(s,w,NULL,NULL,(float)elapsed*0.001f);
        if (!stab_ahrs_up(s)) { s->fault=ATLAS_STAB_FAULT_VECTOR; return; }
        s->timestamp_ms=sample->timestamp_ms; s->ready=true; s->fault=ATLAS_STAB_FAULT_NONE;
        return;
    }
    if (!isfinite(an) || !isfinite(wn) || wn>500.0f || an<0.1f || an>2.0f)
    {
        s->fault=(!isfinite(an)||!isfinite(wn))?ATLAS_STAB_FAULT_NONFINITE:
            (wn>500.0f?ATLAS_STAB_FAULT_RATE:ATLAS_STAB_FAULT_ACCEL);
        s->initialized=s->ready=false; s->still_count=0U; return;
    }
    const bool accel_good=an>=0.85f && an<=1.15f;
    if (!s->initialized || elapsed>ATLAS_STAB_SAMPLE_MAX_MS)
    {
        s->initialized=s->ready=false; s->still_count=0U;
        if (!accel_good) { s->fault=ATLAS_STAB_FAULT_CORRECTION; return; }
        float m0[3];
        const bool have_mag=stab_mag(s,sample,m0);
        stab_ahrs_start(s,a,an,have_mag);
        if(have_mag) s->mag_ms=s->mag_in_ms;
        s->settled_ms=sample->timestamp_ms;
        s->accel_ms=sample->timestamp_ms;
        s->initialized=true;
    }
    else
    {
        /* Fusion integrates the gyro and feeds back only an in-range accelerometer.
         * A zero vector tells it to coast on the gyro alone. */
        float m[3];
        stab_ahrs_step(s,w,accel_good?a:NULL,stab_mag(s,sample,m)?m:NULL,(float)elapsed*0.001f);
        if(accel_good) s->accel_ms=sample->timestamp_ms;
        if(!stab_ahrs_up(s)) { s->fault=ATLAS_STAB_FAULT_VECTOR; s->initialized=s->ready=false; return; }
    }
    s->timestamp_ms=sample->timestamp_ms;
    s->ready=(uint32_t)(s->timestamp_ms-s->settled_ms)>=250U &&
             (uint32_t)(s->timestamp_ms-s->accel_ms)<=250U;
    s->fault=s->ready?ATLAS_STAB_FAULT_NONE:
        ((uint32_t)(s->timestamp_ms-s->accel_ms)>250U?ATLAS_STAB_FAULT_CORRECTION:ATLAS_STAB_FAULT_SETTLING);
    /* Calibration requires stationary samples for a full second, not one reading.
     * Bias up to 10 dps is allowed for this sensor; motion resets the accumulator. */
    bool still=an>=0.9f && an<=1.1f && stab_norm(sample->gyro_dps)<10.0f;
    for(unsigned i=0;i<3U;++i)
        if(s->still_count && (fabsf(a[i]-s->previous_accel[i])>0.04f ||
            fabsf(a[i]-s->sum_accel[i]/(float)s->still_count)>0.025f)) still=false;
    if(!still) s->still_count=0U;
    else
    {
        if(s->still_count==0U)
        {
            memset(s->sum_accel,0,sizeof(s->sum_accel)); memset(s->sum_gyro,0,sizeof(s->sum_gyro));
            s->still_ms=s->timestamp_ms;
        }
        /* A bounded sliding commissioning window avoids long-run sum overflow. */
        if(s->still_count>=300U)
        {
            for(unsigned i=0;i<3U;++i) { s->sum_accel[i]*=0.5f; s->sum_gyro[i]*=0.5f; }
            s->still_count/=2U;
        }
        for(unsigned i=0;i<3U;++i)
        { s->sum_accel[i]+=a[i]; s->sum_gyro[i]+=sample->gyro_dps[i]; }
        ++s->still_count;
    }
    memcpy(s->previous_accel,a,sizeof(a));
}
/** @brief Prepare a disabled, stationary-upright calibration. @param s State.
 * @param reverse Direction bits. @param c Output. @return Commissioning checks passed. */
bool AtlasStabilization_Calibrate(const AtlasStabilization *s,uint32_t reverse,AtlasStabilizationConfig *c)
{
    if(!s->ready || reverse>15U || s->still_count<50U || (uint32_t)(s->timestamp_ms-s->still_ms)<1000U) return false;
    memset(c,0,sizeof(*c)); c->reverse_mask=reverse;
    for(unsigned i=0;i<3U;++i)
    { c->upright[i]=s->sum_accel[i]/(float)s->still_count; c->bias_dps[i]=s->sum_gyro[i]/(float)s->still_count; }
    if(!stab_unit(c->upright)) return false;
    AtlasStabilization_Seal(c);
    return AtlasStabilization_ConfigValid(c);
}
/** @brief Closest reachable down direction for four radial outward shafts. @param c Calibration.
 * @param up Current up. @param previous Last pulses. @param pulse Targets.
 * @param limited Clipping bits. @param singular Unobservable-angle bits. @return Valid geometry. */
bool AtlasStabilization_Targets(const AtlasStabilizationConfig *c,const float up[3],
                                const uint16_t previous[4],uint16_t pulse[4],uint32_t *limited,uint32_t *singular)
{
    if(!AtlasStabilization_ConfigValid(c)) return false;
    float u[3]={up[0],up[1],up[2]},x[3]={0,0,1},y[3];
    if(!stab_unit(u)) return false;
    const float dot=stab_dot(x,c->upright);
    for(unsigned i=0;i<3U;++i) x[i]-=dot*c->upright[i];
    if(!stab_unit(x)) return false;
    stab_cross(c->upright,x,y);
    const float ux=stab_dot(u,x),uy=stab_dot(u,y),uz=stab_dot(u,c->upright);
    const float co[4]={0.7071067812f,-0.7071067812f,-0.7071067812f,0.7071067812f};
    const float si[4]={0.7071067812f,0.7071067812f,-0.7071067812f,-0.7071067812f};
    *limited=*singular=0U;
    for(unsigned i=0;i<4U;++i)
    {
        /* Outward axis r=(cos(a),-sin(a),0); r x resting_down=(sin(a),cos(a),0).
         * Dot this positive-rotation tangent with world_down=(-ux,-uy,-uz). */
        const float tangent=-si[i]*ux-co[i]*uy;
        if(hypotf(tangent,uz)<0.08f)
        { *singular|=1U<<i; pulse[i]=(previous[i]>=1000U && previous[i]<=2000U)?previous[i]:1500U; continue; }
        float angle=atan2f(tangent,uz)/RAD_PER_DEG;
        /* Increasing PWM is assumed clockwise viewed from outside the shaft.
         * Reverse is an explicit commissioning setting for each actual servo. */
        if(!(c->reverse_mask&(1U<<i))) angle=-angle;
        if(fabsf(angle)>50.0f) *limited|=1U<<i;
        /* At inverted orientations, select the same nearest end across atan2's
         * +/-180 branch cut instead of sweeping through the full travel. */
        if(fabsf(angle)>178.0f && previous[i]!=1500U && previous[i]>=1000U && previous[i]<=2000U)
            angle=previous[i]>1500U ? 50.0f : -50.0f;
        if(angle>50.0f) angle=50.0f;
        if(angle<-50.0f) angle=-50.0f;
        pulse[i]=(uint16_t)lroundf(1500.0f+10.0f*angle);
    }
    return true;
}

/** @brief Reset rest detection at the actual neutral start. @param s State. @param now Tick. */
void AtlasStabilization_CommandsReset(AtlasStabilizationCommands *s,uint32_t now)
{
    memset(s,0,sizeof(*s));
    for(unsigned i=0;i<ATLAS_STAB_CHANNELS;++i) { s->anchor_us[i]=1500U; s->quiet_ms[i]=now; }
}
/** @brief Stream current destinations during motion, then freeze small rest noise.
 * @param s Rest state. @param target Raw targets. @param previous Current pulses.
 * @param singular Hold masks. @param now Tick. @param pulse Outputs. @return Input validity. */
bool AtlasStabilization_CommandsStep(AtlasStabilizationCommands *s,const uint16_t target[4],
                                     const uint16_t previous[4],uint32_t singular,uint32_t now,uint16_t pulse[4])
{
    if(!s || !target || !previous || !pulse || singular>15U || s->tracking_mask>15U) return false;
    AtlasStabilizationCommands next=*s;
    uint16_t result[ATLAS_STAB_CHANNELS];
    for(unsigned i=0;i<ATLAS_STAB_CHANNELS;++i)
    {
        if(target[i]<1000U || target[i]>2000U || previous[i]<1000U || previous[i]>2000U ||
           next.anchor_us[i]<1000U || next.anchor_us[i]>2000U) return false;
        const uint32_t bit=1U<<i;
        result[i]=previous[i];
        if(singular&bit)
        {
            next.tracking_mask&=~bit; next.anchor_us[i]=previous[i]; next.quiet_ms[i]=now;
            continue;
        }
        const int change=(int)target[i]-(int)previous[i];
        if(!(next.tracking_mask&bit))
        {
            next.anchor_us[i]=previous[i]; next.quiet_ms[i]=now;
            if(change<(int)ATLAS_STAB_COMMAND_WAKE_US && change>-(int)ATLAS_STAB_COMMAND_WAKE_US &&
               (change==0 || (target[i]!=1000U && target[i]!=2000U))) continue;
            next.tracking_mask|=bit; next.anchor_us[i]=target[i];
        }
        const int excursion=(int)target[i]-(int)next.anchor_us[i];
        if(excursion>=(int)ATLAS_STAB_COMMAND_QUIET_US || excursion<=-(int)ATLAS_STAB_COMMAND_QUIET_US)
        { next.anchor_us[i]=target[i]; next.quiet_ms[i]=now; }
        if((uint32_t)(now-next.quiet_ms[i])>=ATLAS_STAB_COMMAND_QUIET_MS)
            next.tracking_mask&=~bit;
        else result[i]=target[i];
    }
    *s=next;
    memcpy(pulse,result,sizeof(result));
    return true;
}
