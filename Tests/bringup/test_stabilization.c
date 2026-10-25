/** @file test_stabilization.c @brief Independent geometry, estimator and settings regression.
 * Major functions: geometry compares against Rodrigues horn kinematics; estimator
 * checks stationary bias, signed rotation, acceleration rejection and tick wrap. */
#include "atlas_stabilization.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#define PI 3.14159265358979323846f

static AtlasStabilizationConfig config(void)
{
    AtlasStabilizationConfig c={.upright={0,-1,0}};
    AtlasStabilization_Seal(&c); assert(AtlasStabilization_ConfigValid(&c));return c;
}
/* Forward horn geometry is independent of the production inverse expression.
 * Find the best reachable whole-degree angle by brute force in body coordinates. */
static float down_score(unsigned servo,int pulse_angle,const float body_up[3])
{
    const float az=(45.0f+90.0f*(float)servo)*PI/180.0f;
    const float axis[3]={cosf(az),-sinf(az),0};
    const float neutral[3]={0,0,-1};
    const float angle=-(float)pulse_angle*PI/180.0f;
    const float cross[3]={-axis[1],axis[0],0};
    float score=0;
    for(unsigned k=0;k<3;++k) score-=body_up[k]*(neutral[k]*cosf(angle)+cross[k]*sinf(angle));
    return score;
}
static void geometry(void)
{
    AtlasStabilizationConfig c=config(); uint16_t last[4]={1500,1500,1500,1500},p[4];uint32_t lim,sing;
    for(int pitch=-80;pitch<=80;pitch+=10) for(int roll=-80;roll<=80;roll+=10)
    {
        const float x=(float)pitch*PI/180.0f,y=(float)roll*PI/180.0f;
        const float body_up[3]={sinf(x)*cosf(y),sinf(y),cosf(x)*cosf(y)};
        const float sensor_up[3]={-body_up[1],-body_up[2],body_up[0]};
        assert(AtlasStabilization_Targets(&c,sensor_up,last,p,&lim,&sing));
        for(unsigned i=0;i<4;++i)
        {
            assert(p[i]>=1000 && p[i]<=2000);
            if(sing&(1U<<i))continue;
            const int angle=(int)lroundf(((float)p[i]-1500.0f)/10.0f);
            const float actual=down_score(i,angle,body_up);
            for(int test=-50;test<=50;++test) assert(actual+0.0003f>=down_score(i,test,body_up));
        }
    }
    const float up[3]={0,-1,0};assert(AtlasStabilization_Targets(&c,up,last,p,&lim,&sing));
    for(unsigned i=0;i<4;++i)assert(p[i]==1500);
    const float sideways[3]={0.70710678f,0,0.70710678f};
    assert(AtlasStabilization_Targets(&c,sideways,last,p,&lim,&sing));assert((sing&5U)==5U);
    const float tilt[3]={0,-0.8660254f,0.5f};uint16_t reverse[4];
    assert(AtlasStabilization_Targets(&c,tilt,last,p,&lim,&sing));
    c.reverse_mask=15;AtlasStabilization_Seal(&c);
    assert(AtlasStabilization_Targets(&c,tilt,last,reverse,&lim,&sing));
    for(unsigned i=0;i<4;++i)assert(p[i]+reverse[i]==3000U);
    const float bad[3]={NAN,0,0};assert(!AtlasStabilization_Targets(&c,bad,last,p,&lim,&sing));
    puts("PASS stabilization geometry: 289 orientations x 4 shafts x 101 candidate angles; neutral, reversal, clipping, singularity, NaN");
}
static void estimator(void)
{
    AtlasStabilization s={0};AtlasStabilizationSample sample={{0,-1,0},{3,0,0},UINT32_MAX-1500U,true};
    AtlasStabilizationConfig c;
    for(unsigned i=0;i<120;++i){sample.timestamp_ms+=10U;AtlasStabilization_Update(&s,&sample,NULL,sample.timestamp_ms);}
    assert(s.ready);assert(AtlasStabilization_Calibrate(&s,0,&c));assert(fabsf(c.bias_dps[0]-3.0f)<0.001f);
    uint8_t original[64];memcpy(original,&c,64);
    for(unsigned i=0;i<64;++i){((uint8_t*)&c)[i]^=1;assert(!AtlasStabilization_ConfigValid(&c));memcpy(&c,original,64);}
    for(unsigned i=0;i<500;++i){sample.timestamp_ms+=10U;AtlasStabilization_Update(&s,&sample,&c,sample.timestamp_ms);}
    assert(s.ready && s.up[1]<-0.999f); /* Bias-corrected rest survives uint32 wrap. */
    const unsigned count=s.still_count;
    AtlasStabilization_Update(&s,&sample,&c,sample.timestamp_ms);assert(s.still_count==count);
    AtlasStabilization_Update(&s,&sample,&c,sample.timestamp_ms+101U);
    assert(!s.ready && !s.initialized && s.fault==ATLAS_STAB_FAULT_STALE);
    memset(&s,0,sizeof(s));c=config();
    sample=(AtlasStabilizationSample){{0,-1,0},{0,0,0},1000,true};
    for(unsigned i=0;i<40;++i){sample.timestamp_ms+=10;AtlasStabilization_Update(&s,&sample,&c,sample.timestamp_ms);}
    /* +X rotation: a fixed initial -Y up vector rotates toward +Z in sensor axes. */
    for(unsigned i=1;i<=100;++i)
    {
        const float a=(float)i*0.3f*PI/180.0f;
        sample.accel_g[1]=-cosf(a);sample.accel_g[2]=sinf(a);sample.gyro_dps[0]=30;
        sample.timestamp_ms+=10;AtlasStabilization_Update(&s,&sample,&c,sample.timestamp_ms);
    }
    assert(s.ready && fabsf(s.up[2]-0.5f)<0.005f && fabsf(s.up[1]+0.8660254f)<0.005f);
    assert(!AtlasStabilization_Calibrate(&s,0,&c));
    sample.accel_g[0]=NAN;sample.timestamp_ms+=10;AtlasStabilization_Update(&s,&sample,NULL,sample.timestamp_ms);
    assert(!s.ready && s.fault==ATLAS_STAB_FAULT_NONFINITE);
    sample=(AtlasStabilizationSample){{0,-1,0},{0,0,0},20000,false};
    AtlasStabilization_Update(&s,&sample,NULL,sample.timestamp_ms);assert(s.fault==ATLAS_STAB_FAULT_SAMPLE);
    sample.valid=true;sample.gyro_dps[0]=501;
    AtlasStabilization_Update(&s,&sample,NULL,sample.timestamp_ms);assert(s.fault==ATLAS_STAB_FAULT_RATE);
    sample.gyro_dps[0]=0;sample.accel_g[1]=-2.01f;
    AtlasStabilization_Update(&s,&sample,NULL,sample.timestamp_ms);assert(s.fault==ATLAS_STAB_FAULT_ACCEL);
    sample.accel_g[1]=-1;
    for(unsigned i=0;i<30;++i){sample.timestamp_ms+=10;AtlasStabilization_Update(&s,&sample,NULL,sample.timestamp_ms);}
    assert(s.ready && s.fault==ATLAS_STAB_FAULT_NONE);
    sample.accel_g[1]=-1.3f;
    for(unsigned i=0;i<26;++i){sample.timestamp_ms+=10;AtlasStabilization_Update(&s,&sample,NULL,sample.timestamp_ms);}
    assert(!s.ready && s.fault==ATLAS_STAB_FAULT_CORRECTION);
    puts("PASS stabilization estimator: stationary bias, wrap, duplicate/stale/NaN rejection, signed gyro rotation and moving calibration rejection; all 64 record bytes CRC-covered");
}
/* The Fusion AHRS must start exactly at the measured up vector for every orientation
 * (including the 180-degree branch) and, gyro-only, follow the independent u'=-w x u law. */
static void fusion_attitude(void)
{
    for(int el=-90;el<=90;el+=15) for(int az=0;az<360;az+=20)
    {
        const float e=(float)el*PI/180.0f,a=(float)az*PI/180.0f;
        float u[3]={cosf(e)*cosf(a),cosf(e)*sinf(a),sinf(e)};
        if(el==-90) u[0]=u[1]=0.0f;
        AtlasStabilization s={0};AtlasStabilizationSample sample={{u[0],u[1],u[2]},{0,0,0},1000,true};
        AtlasStabilization_Update(&s,&sample,NULL,sample.timestamp_ms);
        assert(s.initialized);
        for(unsigned i=0;i<3;++i) assert(fabsf(s.up[i]-u[i])<2e-3f);
    }
    AtlasStabilization s={0};AtlasStabilizationSample sample={{0.6f,-0.8f,0},{0,0,0},1000,true};
    AtlasStabilization_Update(&s,&sample,NULL,1000);
    float truth[3]={0.6f,-0.8f,0};const float w[3]={20,-35,50};
    sample.accel_g[0]=1.3f; /* out of the 0.85-1.15 g feedback window, still a legal sample */
    memcpy(sample.gyro_dps,w,sizeof(w));
    for(unsigned i=0;i<150;++i)
    {
        sample.timestamp_ms+=4U+(i&1U); /* 240 Hz jitter */
        const float dt=(float)(4U+(i&1U))*0.001f,wn=sqrtf(w[0]*w[0]+w[1]*w[1]+w[2]*w[2]);
        const float ang=wn*PI/180.0f*dt,ax[3]={w[0]/wn,w[1]/wn,w[2]/wn};
        const float cr[3]={ax[1]*truth[2]-ax[2]*truth[1],ax[2]*truth[0]-ax[0]*truth[2],ax[0]*truth[1]-ax[1]*truth[0]};
        const float dot=ax[0]*truth[0]+ax[1]*truth[1]+ax[2]*truth[2];
        for(unsigned k=0;k<3;++k) truth[k]=cosf(ang)*truth[k]-sinf(ang)*cr[k]+(1.0f-cosf(ang))*dot*ax[k];
        AtlasStabilization_Update(&s,&sample,NULL,sample.timestamp_ms);
    }
    for(unsigned k=0;k<3;++k) assert(fabsf(s.up[k]-truth[k])<0.01f);
    {   /* Flags: at rest the accelerometer is used; a sustained 40-degree disagreement
         * is rejected, trips acceleration recovery once the trigger reaches 1.0, then re-admits it. */
        AtlasStabilization f={0};AtlasStabilizationSample r={{0,-1,0},{0,0,0},1000,true};
        for(unsigned i=0;i<20;++i){r.timestamp_ms+=4;AtlasStabilization_Update(&f,&r,NULL,r.timestamp_ms);}
        assert(!(f.flags&(ATLAS_STAB_FLAG_ACCEL_IGNORED|ATLAS_STAB_FLAG_LAUNCHED|ATLAS_STAB_FLAG_STARTUP)) && f.accel_trigger==0.0f);
        r.accel_g[0]=0.64f;r.accel_g[1]=-0.77f; /* ~40 degrees off, in the 0.85-1.15 g window */
        r.timestamp_ms+=4;AtlasStabilization_Update(&f,&r,NULL,r.timestamp_ms);
        assert((f.flags&ATLAS_STAB_FLAG_ACCEL_IGNORED) && f.accel_error_deg>30.0f && f.accel_trigger>0.0f && f.accel_trigger<1.0f);
        bool recovered=false;
        for(unsigned i=0;i<2400 && !recovered;++i){r.timestamp_ms+=4;AtlasStabilization_Update(&f,&r,NULL,r.timestamp_ms);recovered=f.flags&ATLAS_STAB_FLAG_ACCEL_RECOVERY;}
        assert(recovered && f.accel_trigger>=1.0f);
    }
    puts("PASS stabilization Fusion attitude: exact start over 234 orientations; 240 Hz jittered gyro-only rotation matches Rodrigues");
}
/* After launch the accelerometer is ignored: boost acceleration, a gap and a flipped
 * accelerometer must not move the gyro-only attitude away from the Rodrigues truth. */
static void launch_coast(void)
{
    AtlasStabilization s={0};AtlasStabilizationSample sample={{0,-1,0},{0,0,0},1000,true};
    for(unsigned i=0;i<60;++i){sample.timestamp_ms+=5;AtlasStabilization_Update(&s,&sample,NULL,sample.timestamp_ms);}
    assert(s.ready && !s.launched);
    sample.accel_g[1]=-2.5f;sample.timestamp_ms+=5;AtlasStabilization_Update(&s,&sample,NULL,sample.timestamp_ms);
    assert(!s.launched && s.fault==ATLAS_STAB_FAULT_ACCEL); /* below launch threshold: legacy reset */
    for(unsigned i=0;i<60;++i){sample.accel_g[1]=-1;sample.timestamp_ms+=5;AtlasStabilization_Update(&s,&sample,NULL,sample.timestamp_ms);}
    assert(s.ready);
    float truth[3]={0,-1,0};const float w[3]={0,0,90};
    sample.gyro_dps[2]=w[2];
    for(unsigned i=0;i<120;++i)
    {
        sample.accel_g[0]=5.0f;sample.accel_g[1]=-8.0f;sample.accel_g[2]=2.0f; /* boost: ~9.6 g, wrong direction */
        sample.timestamp_ms+=5;if(i==60)sample.timestamp_ms+=300; /* sample gap > 100 ms */
        const float dt=(i==60?305.0f:5.0f)*0.001f,ang=w[2]*PI/180.0f*dt;
        const float x=truth[0],y=truth[1];
        truth[0]=cosf(ang)*x+sinf(ang)*y;truth[1]=-sinf(ang)*x+cosf(ang)*y; /* u' = -w x u about +Z */
        AtlasStabilization_Update(&s,&sample,NULL,sample.timestamp_ms);
        if(i>=2)assert(s.launched && s.ready && s.fault==ATLAS_STAB_FAULT_NONE);
    }
    for(unsigned k=0;k<3;++k) assert(fabsf(s.up[k]-truth[k])<0.01f);
    sample.valid=false;AtlasStabilization_Update(&s,&sample,NULL,sample.timestamp_ms);
    assert(s.launched && s.initialized); /* invalid sample never restarts from the accelerometer */
    assert(s.flags&ATLAS_STAB_FLAG_LAUNCHED && s.flags&ATLAS_STAB_FLAG_ACCEL_IGNORED && !(s.flags&ATLAS_STAB_FLAG_STARTUP));
    AtlasStabilization n={0};assert(!AtlasStabilization_Launch(&n));
    puts("PASS stabilization launch: 3-sample high-g latch, accelerometer ignored in boost, gap and invalid samples, gyro-only attitude matches truth");
}
/* Independent truth for heading: build the sensor frame in NWU earth coordinates from a chosen heading of the
 * +Z axis (clockwise from north) and a nose-down-free upright board (up = -Y), then express gravity and an
 * Earth field of 0.2 G north / 0.4 G down in that frame. */
static void mag_frame(float heading_deg,float up_s[3],float mag_s[3])
{
    const float h=heading_deg*PI/180.0f;
    const float z[3]={cosf(h),-sinf(h),0},y[3]={0,0,-1};
    const float x[3]={y[1]*z[2]-y[2]*z[1],y[2]*z[0]-y[0]*z[2],y[0]*z[1]-y[1]*z[0]};
    const float* axis[3]={x,y,z},field[3]={0.2f,0,-0.4f};
    for(unsigned k=0;k<3;++k){up_s[k]=axis[k][2];mag_s[k]=axis[k][0]*field[0]+axis[k][1]*field[1]+axis[k][2]*field[2];}
}
#if !ATLAS_STAB_USE_MAGNETOMETER
/* Default build: a perfect field is supplied and must be ignored. Up is unchanged, heading is never
 * referenced to north, Fusion's startup ramp is skipped, and the flags say why. */
static void magnetometer(void)
{
    float u[3],m[3];mag_frame(90,u,m);
    AtlasStabilization s={0};AtlasStabilizationSample x={{u[0],u[1],u[2]},{0,0,0},1000,true};
    AtlasStabilizationMagnetometer g={{m[0],m[1],m[2]},1000,true};
    for(unsigned i=0;i<1500;++i)
    {   x.timestamp_ms+=4;g.timestamp_ms=x.timestamp_ms;AtlasStabilization_SetMagnetometer(&s,&g);AtlasStabilization_Update(&s,&x,NULL,x.timestamp_ms);
        assert(!(s.flags&ATLAS_STAB_FLAG_STARTUP) && (s.flags&ATLAS_STAB_FLAG_MAG_IGNORED));
        assert(!(s.flags&ATLAS_STAB_FLAG_HEADING_VALID) && !s.mag_locked && s.mag_trigger==0.0f && s.mag_error_deg==0.0f);}
    assert(fabsf(s.up[1]+1.0f)<0.001f);
    puts("PASS stabilization magnetometer disabled (default): field ignored, no heading lock, startup skipped, up unchanged");
}
#else
static float angle_diff(float a,float b){float d=fmodf(a-b+540.0f,360.0f)-180.0f;return fabsf(d);}
static void magnetometer(void)
{
    const float headings[]={0,45,90,200,333};
    for(unsigned n=0;n<5;++n)
    {
        float u[3],m[3];mag_frame(headings[n],u,m);
        AtlasStabilization s={0};
        AtlasStabilizationSample x={{u[0],u[1],u[2]},{0,0,0},1000,true};
        AtlasStabilizationMagnetometer g={{m[0],m[1],m[2]},1000,true};
        for(unsigned i=0;i<1500;++i)
        {   x.timestamp_ms+=4;if(i%25==0){g.timestamp_ms=x.timestamp_ms;AtlasStabilization_SetMagnetometer(&s,&g);} /* 240 Hz IMU, ~10 Hz field */
            AtlasStabilization_Update(&s,&x,NULL,x.timestamp_ms);}
        assert(s.flags&ATLAS_STAB_FLAG_HEADING_VALID && s.mag_locked && !(s.flags&ATLAS_STAB_FLAG_STARTUP));
        assert(angle_diff(s.heading_deg,headings[n])<1.0f);
        assert(fabsf(s.up[1]+1.0f)<0.01f); /* Heading feedback must not move gravity. */
    }
    /* Wrong-magnitude and stale/absent fields are rejected and flagged; heading is not referenced. */
    float u[3],m[3];mag_frame(90,u,m);
    AtlasStabilization s={0};AtlasStabilizationSample x={{u[0],u[1],u[2]},{0,0,0},1000,true};
    AtlasStabilizationMagnetometer g={{m[0]*5,m[1]*5,m[2]*5},1000,true};
    for(unsigned i=0;i<500;++i){x.timestamp_ms+=4;g.timestamp_ms=x.timestamp_ms;AtlasStabilization_SetMagnetometer(&s,&g);AtlasStabilization_Update(&s,&x,NULL,x.timestamp_ms);}
    assert((s.flags&ATLAS_STAB_FLAG_MAG_IGNORED) && !(s.flags&ATLAS_STAB_FLAG_HEADING_VALID) && !s.mag_locked);
    /* A 90-degree wrong field is rejected once lock and startup are over; recovery re-admits it after the timeout. */
    mag_frame(90,u,m);memset(&s,0,sizeof(s));
    x=(AtlasStabilizationSample){{u[0],u[1],u[2]},{0,0,0},1000,true};g=(AtlasStabilizationMagnetometer){{m[0],m[1],m[2]},1000,true};
    for(unsigned i=0;i<1500;++i){x.timestamp_ms+=4;g.timestamp_ms=x.timestamp_ms;AtlasStabilization_SetMagnetometer(&s,&g);AtlasStabilization_Update(&s,&x,NULL,x.timestamp_ms);}
    assert(s.mag_locked && angle_diff(s.heading_deg,90)<1.0f);
    mag_frame(0,u,m);for(unsigned k=0;k<3;++k)g.gauss[k]=m[k]; /* field now says heading 0 while gyro says 90 */
    bool ignored=false,recovered=false;
    for(unsigned i=0;i<4000 && !recovered;++i)
    {   x.timestamp_ms+=4;g.timestamp_ms=x.timestamp_ms;AtlasStabilization_SetMagnetometer(&s,&g);AtlasStabilization_Update(&s,&x,NULL,x.timestamp_ms);
        ignored|=(s.flags&ATLAS_STAB_FLAG_MAG_IGNORED)&&s.mag_error_deg>20.0f;recovered=s.flags&ATLAS_STAB_FLAG_MAG_RECOVERY;}
    assert(ignored && recovered);
    /* Launch: the field is no longer used; a flipped field and boost acceleration leave a gyro-only heading alone. */
    mag_frame(200,u,m);memset(&s,0,sizeof(s));
    x=(AtlasStabilizationSample){{u[0],u[1],u[2]},{0,0,0},1000,true};g=(AtlasStabilizationMagnetometer){{m[0],m[1],m[2]},1000,true};
    for(unsigned i=0;i<1500;++i){x.timestamp_ms+=4;g.timestamp_ms=x.timestamp_ms;AtlasStabilization_SetMagnetometer(&s,&g);AtlasStabilization_Update(&s,&x,NULL,x.timestamp_ms);}
    assert(s.flags&ATLAS_STAB_FLAG_HEADING_VALID);
    x.accel_g[0]=5;x.accel_g[1]=-8;x.accel_g[2]=2;for(unsigned k=0;k<3;++k)g.gauss[k]=-m[k];
    for(unsigned i=0;i<250;++i){x.timestamp_ms+=4;g.timestamp_ms=x.timestamp_ms;AtlasStabilization_SetMagnetometer(&s,&g);AtlasStabilization_Update(&s,&x,NULL,x.timestamp_ms);}
    assert(s.launched && (s.flags&ATLAS_STAB_FLAG_MAG_IGNORED) && (s.flags&ATLAS_STAB_FLAG_HEADING_VALID));
    assert(angle_diff(s.heading_deg,200)<1.0f);
    /* A +Z axis pointing straight up has no heading. */
    AtlasStabilization v={0};AtlasStabilizationSample z={{0,0,1},{0,0,0},1000,true};
    AtlasStabilizationMagnetometer h={{0.2f,0,-0.4f},1000,true};
    for(unsigned i=0;i<300;++i){z.timestamp_ms+=4;h.timestamp_ms=z.timestamp_ms;AtlasStabilization_SetMagnetometer(&v,&h);AtlasStabilization_Update(&v,&z,NULL,z.timestamp_ms);}
    assert(!(v.flags&ATLAS_STAB_FLAG_HEADING_VALID));
    puts("PASS stabilization magnetometer: heading within 1 degree for 5 headings against independent NWU truth, gravity unmoved, bad/stale fields rejected, recovery, frozen after launch, vertical axis has no heading");
}
#endif
static void motion(void)
{
    AtlasStabilizationCommands s;
    uint32_t now=UINT32_MAX-37U;
    AtlasStabilization_CommandsReset(&s,now);
    uint16_t p[4]={1500,1500,1500,1500},t[4];
    /* Fifty seconds of rest noise across tick wrap produces no micro-moves. */
    for(unsigned n=0;n<10000;++n)
    {
        now+=5;
        for(unsigned i=0;i<4;++i)t[i]=(uint16_t)(1495+(n+i*3)%11);
        assert(AtlasStabilization_CommandsStep(&s,t,p,0,now,p));
        for(unsigned i=0;i<4;++i)assert(p[i]==1500);
        assert(!s.tracking_mask);
    }
    /* Full range moves and reversals are direct, with no software speed limit. */
    for(unsigned phase=0;phase<2;++phase)
    {
        now+=5;
        const uint16_t end=phase?1000:2000;
        for(unsigned i=0;i<4;++i)t[i]=end;
        assert(AtlasStabilization_CommandsStep(&s,t,p,0,now,p));
        for(unsigned i=0;i<4;++i)assert(p[i]==end);
    }
    for(unsigned i=0;i<4;++i)p[i]=1500;
    AtlasStabilization_CommandsReset(&s,now);
    /* Once awakened, even single-microsecond target changes pass every 5 ms.
     * Opposite channels must track independently and a stationary one stays put. */
    for(unsigned n=1;n<=500;++n)
    {
        now+=5;t[0]=t[2]=(uint16_t)(1500+n);t[1]=(uint16_t)(1500-n);t[3]=(uint16_t)(1499+n%3);
        assert(AtlasStabilization_CommandsStep(&s,t,p,0,now,p));
        for(unsigned i=0;i<3;++i)assert(p[i]==(n<6?1500:t[i]));
        assert(p[3]==1500);
    }
    /* Follow a continuous small reversal, then settle and ignore +/-1 us noise. */
    for(unsigned n=1;n<=100;++n)
    {
        now+=5;t[0]=t[2]=(uint16_t)(2000-n);t[1]=(uint16_t)(1000+n);
        assert(AtlasStabilization_CommandsStep(&s,t,p,0,now,p));
        for(unsigned i=0;i<3;++i)assert(p[i]==t[i]);
    }
    for(unsigned n=0;n<40;++n)
    { now+=5;assert(AtlasStabilization_CommandsStep(&s,t,p,0,now,p)); }
    assert(s.tracking_mask==0);
    uint16_t held[4];memcpy(held,p,sizeof(held));
    for(unsigned n=0;n<200;++n)
    {
        now+=5;for(unsigned i=0;i<4;++i)t[i]=(uint16_t)(held[i]-1+n%3);
        assert(AtlasStabilization_CommandsStep(&s,t,p,0,now,p));
        assert(memcmp(held,p,sizeof(held))==0 && s.tracking_mask==0);
    }
    /* Arbitrarily slow movement accumulates against the last actual output.
     * It may remain inside the rest window, but cannot accumulate larger error. */
    AtlasStabilization_CommandsReset(&s,now);
    for(unsigned i=0;i<4;++i)p[i]=1500;
    for(unsigned n=1;n<=500;++n)
    {
        now+=200;for(unsigned i=0;i<4;++i)t[i]=(uint16_t)(1500+n);
        assert(AtlasStabilization_CommandsStep(&s,t,p,0,now,p));
        for(unsigned i=0;i<4;++i)assert(p[i]<=t[i] && t[i]-p[i]<6);
    }
    /* A sub-window endpoint correction remains reachable even from rest. */
    AtlasStabilization_CommandsReset(&s,now);
    for(unsigned i=0;i<4;++i){p[i]=(i&1)?1981:1023;t[i]=(i&1)?2000:1000;}
    now+=5;assert(AtlasStabilization_CommandsStep(&s,t,p,0,now,p));
    for(unsigned i=0;i<4;++i)assert(p[i]==t[i]);
    AtlasStabilization_CommandsReset(&s,now);p[0]=1002;t[0]=1000;
    now+=5;assert(AtlasStabilization_CommandsStep(&s,t,p,0,now,p));assert(p[0]==1000);
    /* Singular channels freeze immediately and discard their tracking state. */
    for(unsigned i=0;i<4;++i)t[i]=1000;
    p[0]=1637;
    for(unsigned n=0;n<50;++n)
    {
        now+=5;assert(AtlasStabilization_CommandsStep(&s,t,p,1,now,p));
        assert(p[0]==1637 && !(s.tracking_mask&1));
    }
    /* A late iteration uses only the current target, without a catch-up ramp. */
    now+=1000;t[0]=1800;assert(AtlasStabilization_CommandsStep(&s,t,p,0,now,p));assert(p[0]==1800);
    /* Reproduce both observed native references with no additional ramp/lag. */
    for(unsigned period=1000;period<=2000;period+=1000)
    {
        AtlasStabilization_CommandsReset(&s,now);
        for(unsigned i=0;i<4;++i)p[i]=1500;
        for(unsigned elapsed=5;elapsed<=period;elapsed+=5)
        {
            now+=5;
            const uint16_t reference=(uint16_t)lroundf(1500.0f+500.0f*sinf(6.28318530718f*(float)elapsed/(float)period));
            for(unsigned i=0;i<4;++i)t[i]=reference;
            assert(AtlasStabilization_CommandsStep(&s,t,p,0,now,p));
            for(unsigned i=0;i<4;++i)assert(p[i]==reference);
        }
    }
    /* Validation failure cannot partly mutate either commands or rest state. */
    AtlasStabilizationCommands before=s;
    uint16_t result[4]={11,22,33,44};t[3]=999;
    assert(!AtlasStabilization_CommandsStep(&s,t,p,0,now,result));
    assert(result[0]==11 && result[3]==44 && memcmp(&s,&before,sizeof(s))==0);
    t[3]=1000;p[2]=0;
    assert(!AtlasStabilization_CommandsStep(&s,t,p,0,now,result));
    assert(result[0]==11 && result[3]==44 && memcmp(&s,&before,sizeof(s))==0);
    assert(!AtlasStabilization_CommandsStep(NULL,t,p,0,now,result));
    puts("PASS stabilization commands: 200Hz direct tracking, rest-noise hold, independent shafts, reversals, bounded slow-tilt error, endpoints, singularity, late/wrap timing and failure atomicity");
}
int main(void){geometry();estimator();fusion_attitude();launch_coast();magnetometer();motion();return 0;}
