const jscad = require('@jscad/modeling');
const { geom2, path2 } = require('@jscad/modeling').geometries;
const { line,cylinder } = require('@jscad/modeling').primitives;
const { subtract } = jscad.booleans;
const { extrudeLinear } = jscad.extrusions;

const cycloidalCurve = (p,t) => {
	var pcx = p.RPCDr * Math.cos(t);
	var pcy = -p.RPCDr * Math.sin(t);

	var ppx = Math.cos(p._planetXt*t);
	var ppy = Math.sin(p._planetYt*t);
	var atanT = Math.atan(ppy/(p._planetRatio-ppx));
	
	x = pcx - (p.Rr*Math.cos(t+atanT))-(p.E*Math.cos(p.N*t));
	y = pcy + (p.Rr*Math.sin(t+atanT))+(p.E*Math.sin(p.N*t));
	
	return [x,y];
}
const cycloidalDisk = (p) => {
	p._planetXt = 1-p.N;
	p._planetYt = 1-p.N;
	p._planetRatio = p.RPCDr/(p.E*p.N);
	p.retry = p.retry ?? 4;
	p.tolerance = p.tolerance ?? 0.02;
	p.distance = p.distance ?? 0.08;
	var pts = [];
	var pt;
	var t = 0;
	var inc = (2*Math.PI)/p.D;
	var ptP = null;
	var incP;
	var tP;
	var tryCnt = 0;
	while(t < 2*Math.PI) {
		tryCnt++;
		pt = cycloidalCurve(p,t);
		if(ptP != null && tryCnt <= p.retry) {
			var dx = pt[0]-ptP[0];
			var dy = pt[1]-ptP[1];
			var d = Math.sqrt(dx*dx+dy*dy);
			if(d < p.distance-p.tolerance) {
				inc *= 1.2;
				t = tP + inc;
				continue;
			}
			if(d > p.distance+p.tolerance) {
				inc *= 0.8;
				t = tP + inc;
				continue;
			}
		}
		tryCnt = 0;
		pts.push(pt);
		ptP = pt;
		incP = inc;
		tP = t;
		t += inc;
	}
	const cycloidalPath = path2.fromPoints({closed:true},pts.reverse());
	const cycloidalShape = extrudeLinear({height:p.H}, cycloidalPath);
	const centerHole = cylinder({height:p.H*2, radius:p.Cr, segments:Math.trunc(p.D/10)});
	var diskShape = subtract(cycloidalShape,centerHole);
	for(var n = 0; n < p.On; n++) {
		const cr = n * ((2*Math.PI)/p.On);
		const cx = Math.cos(cr) * p.OPCDr;
		const cy = -Math.sin(cr) * p.OPCDr;
		const outputHole = cylinder({center:[cx,cy,0], height:p.H*2, radius:p.Or+p.E, segments:Math.trunc(p.D/10)});
		diskShape = subtract(diskShape,outputHole);
	}
	return diskShape;
}

const main = () => {
	var param = {
		// cycroidal parameters
		N: 34,		// number of rollers
		Rr: 2,		// rasius of roller
		RPCDr: 33,	// radius of rollers PCD
		E: 0.6,		// offset of cycroidal disk
		H: 5,		// height of disk

		// center hole parameters
		Cr: 13,		// radius of center hole

		// output hole parameters
		Or: 4,		// radius of output shaft
		On: 6,		// number of output shaft
		OPCDr: 20,	// radius of output shaft PCD

		// STL quality parameters
		D: 1440,	// number of shape segments
	};
	return cycloidalDisk(param);
}
 
module.exports = { main }
