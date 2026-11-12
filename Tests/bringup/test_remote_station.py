"""Remote wire and gateway boundary tests; no serial device is opened."""
import binascii
import copy
import json
from pathlib import Path
import struct
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'tools/bringup'))
import demo
from protocol import Decoder,Session,validate,valid_command
from remote_telemetry import STATS,decode_snapshot
from ground_station import Station
import flight_log
import tempfile

def hello():
    h=demo.hello();h['uid']=[1,2,3];return h

def sample():
    w=[0]*128
    w[0:5]=[0x31534c54,2000,0x010500,1,255]
    w[17:21]=[10]*4;w[29:33]=[1980]*4
    w[36:39]=[100,(-200)&0xffffffff,1000]
    w[70:81]=[1900,50,3|(1<<8)|(12<<16),340000000,(-1170000000)&0xffffffff,150000,1000,123456,20,1000000,0]
    w[81:87]=[1990,30,256,1023,3300,28];w[87:97]=[3300,8200,5000,8200,0,1,2,3,4,5]
    w[97]=12345|(54321<<16);w[123]=1
    return w


def flight_sample():
    """Pad-ready flight-image snapshot packed as App/Src/atlas_flight_app.c does."""
    w=[0]*128
    w[0:4]=[0x31534c54,2000,0x020200,2]
    w[5:8]=[0,0,1000];w[10]=1000;w[12]=4000           # ADXL pad mean (mg), nose +Z, still 4 s
    w[17:21]=[400,900,5000,480];w[29:33]=[1999]*4
    w[33:36]=[0,0,1000];w[36:39]=[0,0,1000];w[46]=101325
    w[48:52]=[0,0,4000,0];w[55]=1000
    w[56]=1|2|(1<<8)|(1<<19)|(1<<20)                  # started, ready, canards DISABLED, qualified, permitted
    w[105]=0;w[106]=0xC3<<17
    w[111]=1500|(1500<<16);w[114]=1500|(1500<<16)
    w[118]=0x1f|(100<<8);w[119]=4000
    w[70:78]=[1900,0,3|(1<<8)|(12<<16),340000000,(-1170000000)&0xffffffff,150000,1000,123456]  # GPS 3D fix
    w[115]=1|2|4|(3<<16);w[116]=1200;w[117]=1;w[123]=2      # SD: logging FLT0003.BIN
    return w


def flight_remote():
    f=remote();f['payload_hex']=encode(flight_sample());return f


def encode(w):
    body=struct.pack('<127I',*w[:127]);return (body+struct.pack('<I',binascii.crc32(body))).hex()


def remote():
    stats={k:0 for k in STATS};stats.update(good_batches=1,rx_packets=10,rx_bytes=1000,expected_packets=10)
    return dict(type='remote',schema=1,ms=3000,owner_ms=2999,gateway=hello()['uid'],peer=[7,8,9],boot=[10,11],
                sequence=1,received_ms=2900,assembly_ms=330,available=1,tx_ready=1,streaming=1,
                uart_errors=0,uart_dropped=0,stats=stats,payload_hex=encode(sample()))


class RemoteTests(unittest.TestCase):
    def test_checked_units_and_signed_fields(self):
        d=validate(remote())['data']
        self.assertEqual(d['lsm']['mg'],[100,-200,1000]);self.assertEqual(d['power']['raw'][:2],[12345,54321])
        self.assertEqual(d['gnss']['lon_e7'],-1170000000);self.assertEqual(d['gnss']['sv'],12)
        self.assertEqual(d['gnss']['fix'],3);self.assertEqual(d['version'],'1.5.0')

    def test_every_payload_byte_checked_again(self):
        f=remote();raw=bytes.fromhex(f['payload_hex'])
        for i in range(512):
            bad=bytearray(raw);bad[i]^=1
            with self.assertRaises(ValueError):decode_snapshot(bad.hex())

    def test_envelope_not_coercible(self):
        f=remote()
        for key in ['available','ms','sequence','schema','tx_ready']:
            for value in [True,-1,'1',None,2**32]:
                bad=copy.deepcopy(f);bad[key]=value
                with self.assertRaises(ValueError):validate(bad)
        for key in STATS:
            bad=copy.deepcopy(f);bad['stats'][key]=False
            with self.assertRaises(ValueError):validate(bad)
        for key in ['peer','boot','gateway']:
            bad=copy.deepcopy(f);bad[key]=[1]
            with self.assertRaises(ValueError):validate(bad)

    def test_semantic_corruption_even_with_valid_crc(self):
        for offset,value in [(0,1),(3,3),(5,14),(25,20),(56,4),(72,6),(83,512),(84,1024),(106,0x80000000),(107,7<<16),(115,14<<8),(123,2),(124,1)]:
            w=sample();w[offset]=value
            with self.assertRaises(ValueError):decode_snapshot(encode(w))

    def test_flight_snapshot_decodes(self):
        d=validate(flight_remote())['data'];f=d['flight']
        self.assertEqual(d['profile'],'flight');self.assertEqual(d['version'],'2.2.0')
        self.assertEqual((f['phase'],f['cal_ready'],f['cal_progress'],f['outputs_permitted']),(0,1,100,1))
        self.assertEqual(f['adxl_pad_mg'],[0,0,1000]);self.assertEqual(f['canard_state'],1)
        self.assertEqual(f['pwm_mask'],0xC3);self.assertEqual([f['pulse_us'][i] for i in (6,7,0,1)],[1500]*4)
        self.assertEqual(d['adxl']['mg'],[0,0,1000]);self.assertEqual(d['baro']['pa'],101325)
        self.assertEqual((d['gnss']['fix'],d['gnss']['sv'],d['gnss']['lon_e7']),(3,12,-1170000000))
        self.assertEqual((f['apogee_ms'],f['roll_canard']),(0,0))
        self.assertEqual(d['sd'],dict(card=1,mounted=1,logging=1,file=3,written=1200,failed=1,dropped=2))
        w=flight_sample();w[48]=2;w[49]=2;w[52]=(-150)&0xffffffff
        f=decode_snapshot(encode(w))['flight'];self.assertEqual((f['phase'],f['altitude_cm']),(2,-150))
        w=flight_sample();w[48]=4;w[49]=5;w[14]=30000;w[15]=3
        f=decode_snapshot(encode(w))['flight'];self.assertEqual((f['phase'],f['reason'],f['apogee_ms'],f['roll_canard']),(4,5,30000,3))

    def test_flight_snapshot_rejects_impossible_fields(self):
        for offset,value in [(48,5),(49,7),(15,4),(115,8),(72,6),(72,1<<24),(56,1<<21),(56,6<<8),(118,1<<5),(118,101<<8),(13,2),(25,14),(106,1),(124,1)]:
            w=flight_sample();w[offset]=value
            with self.assertRaises(ValueError):decode_snapshot(encode(w))

    def test_sd_flight_log_converts_and_skips_damaged_records(self):
        good=bytes.fromhex(encode(flight_sample()))
        w=flight_sample();w[48]=2;later=bytes.fromhex(encode(w))
        bad=bytearray(good);bad[100]^=1
        with tempfile.TemporaryDirectory() as d:
            src=Path(d)/'FLT0001.BIN';src.write_bytes(good+bytes(bad)+later+good[:100])
            n,damaged=flight_log.convert(src,Path(d)/'out.csv')
            rows=(Path(d)/'out.csv').read_text().splitlines()
        self.assertEqual((n,damaged),(2,2));self.assertEqual(len(rows),3)
        self.assertIn('flight.phase',rows[0].split(','));self.assertIn('gnss.lat_e7',rows[0].split(','))

    def test_remote_cannot_refresh_or_replace_local_status(self):
        s=Session();h=hello();h.update(remote_telemetry=1)
        s.accept(h,1.0)
        local=demo.status();s.accept(local,1.0)
        f=remote();f['data']={'gpio':{'pwm':255}}
        s.accept(f,10.0)
        self.assertFalse(s.fresh(10.0));self.assertEqual(s.status,local)
        self.assertEqual(s.remote['data']['gpio']['pwm'],0)
        self.assertIsNone(s.last_reply);self.assertFalse(s.blocked)

    def test_crc_failure_isolated_from_local_command_session(self):
        f=remote();f['payload_hex']='f'+f['payload_hex'][1:]
        d=Decoder();frames=d.feed((json.dumps(f)+'\n'+json.dumps(demo.status())+'\n').encode())
        self.assertEqual(d.errors,0);self.assertEqual(d.remote_errors,1)
        self.assertEqual([x['type'] for x in frames],['status'])

    def test_gateway_identity_and_legacy_gate(self):
        s=Session();h=hello();s.accept(h,1)
        with self.assertRaises(ValueError):s.accept(remote(),1)
        h['remote_telemetry']=1;s.accept(h,1)
        f=remote();f['gateway']=[1,3,5]
        with self.assertRaises(ValueError):s.accept(f,1)
        for command in ['telemetry on','telemetry off']:
            self.assertTrue(valid_command(command))
            with self.assertRaises(ValueError):Session().request(command,1,True)
        self.assertFalse(valid_command('telemetry on gpio 1'))

    def test_source_cannot_forge_snapshot_data(self):
        f=remote();f['data']={'gnss':{'lat_e7':9}}
        self.assertEqual(validate(f)['data']['gnss']['lat_e7'],340000000)

    def test_empty_snapshot_and_consistency(self):
        f=remote();f['payload_hex']='0'*1024
        with self.assertRaises(ValueError):validate(f)
        f['available']=0;self.assertIsNone(validate(f)['data'])
        f=remote();f['stats']['missing_packets']=11
        with self.assertRaises(ValueError):validate(f)

    def test_station_exposes_separate_compact_readonly_data(self):
        st=Station();h=hello();h.update(remote_telemetry=1)
        st.ingest(h,1);st.ingest(demo.status(),1);st.ingest(remote(),2)
        view=st.snapshot(compact=True)
        self.assertEqual(view['remote']['peer'],[7,8,9]);self.assertEqual(view['status']['type'],'status')
        st.disconnect();self.assertIsNone(st.snapshot()['remote'])

if __name__=='__main__':
    if '--fixture' in sys.argv:
        h=hello();h['remote_telemetry']=1
        local=demo.status();local.update(ms=3000,owner_ms=3000)
        print(json.dumps(dict(mode='live',fresh=True,hello=h,status=local,remote=validate(remote()),
                              flight_remote=validate(flight_remote()),
                              remote_age_ms=20,age_ms=20,pending=None,batch=[],updating=False,blocked='',port='COM3')))
    else:unittest.main(verbosity=2)
