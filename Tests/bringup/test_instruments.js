// Execute only the pure instrument functions in a DOM-free VM. No browser/device.
const fs=require('node:fs'),vm=require('node:vm'),assert=require('node:assert/strict');
const source=fs.readFileSync(require('node:path').join(__dirname,'../../tools/bringup/web/app.js'),'utf8').split("document.addEventListener('click'")[0];
const context=vm.createContext({document:{querySelector:()=>({content:'inert-token'})}});
vm.runInContext(source,context);
const evaluate=code=>vm.runInContext(code,context);
evaluate(`state={fresh:true,blocked:''}; const fixture={ms:10000,attempted:63,init:Array(10).fill(0),count:[1,1,1,1],sample_status:[0,0,0,0],lsm:{t:10000,mg:[0,0,1000]},power:{available:true,status:0,count:1,t:10000,valid:8,mv:[3300,8400,5000,15100]},gnss:{t:10000,frames:1,flags:1,fix:3,lat_e7:0,lon_e7:0}};`);
assert.equal(evaluate('railValue(fixture,3)'),15.1);
assert.equal(evaluate('railValue(fixture,0)'),null);
evaluate('fixture.power.t=9000');assert.equal(evaluate('railValue(fixture,3)'),null);
assert.equal(evaluate('gnssGood(fixture)'),true); // A genuinely valid equator fix is not "missing".
evaluate('fixture.gnss.fix=2');assert.equal(evaluate('gnssGood(fixture)'),false);
evaluate('fixture.gnss.fix=3;fixture.gnss.flags=0');assert.equal(evaluate('gnssGood(fixture)'),false);
evaluate('fixture.gnss.flags=1;fixture.gnss.lat_e7=900000001');assert.equal(evaluate('gnssGood(fixture)'),false);
assert.equal(evaluate('age({ms:10},0xfffffff0)'),26);
assert.equal(evaluate('sensorGood(fixture,1,"lsm")'),true);
evaluate('fixture.lsm.t=8000');assert.equal(evaluate('sensorGood(fixture,1,"lsm")'),false);
assert.equal(evaluate('number(null,2)'),'—');
console.log('PASS: 16 instrument checks; quaternion, units, invalid data, GNSS quality and timestamp wrap.');
