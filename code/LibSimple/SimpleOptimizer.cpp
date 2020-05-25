// Powell Brent Optimizer Based on Numerical Recipes.

#include "SimpleOptimizer.h"


#define MAX_PARAMETERS 50

#include <limits>
#include <cmath>

namespace Simple
{

	// replaces std::isfinite
	inline bool isfinite_impl(double x)
	{
		if (x!=std::numeric_limits<double>::infinity() && x!=-std::numeric_limits<double>::infinity()
			&& x!=std::numeric_limits<double>::quiet_NaN() && x!=std::numeric_limits<double>::signaling_NaN() )
				return true;
		else
			return false;
	}

	int _powell_ncom;
	double _powell_pcom[MAX_PARAMETERS], _powell_xicom[MAX_PARAMETERS];
	double (*_powellFunc)(double*);

	void _powell_linmin(double *p, double *xi, int n, double *fret);
	double _powell_brent(double ax, double bx, double cx, double tol, double *xmin);
	double _powell_f1dim(double x);
	void _powell_mnbrak(double *ax, double *bx, double *cx, double *fa, double *fb, double *fc);

	double _powell_sqr(double x) {return (x*x);}
	void _powell_shft(double *a, double *b, double *c, double *d) {*a=*b; *b=*c; *c=*d;}
	double _powell_sign(double a, double b) {return((b>=0.0) ? std::abs(a):-std::abs(a));}
	double _powell_fmax(double a, double b) {return((a>b) ? a:b);}
	
	// Algorithm as in NRC
	int Optimizer::powell(
		double *p,		// solution
		double *delta,	// step (for example vector of all ones)
		int parNr,		// problem size
		double ftol,	// output tolerance
		int *iterNr,	// output iterations
		double *fret,	// returned function value
		double (*_fun)(double*) )
	{
		int i, j, ibig, iterMax, fixed[MAX_PARAMETERS];
		double xi[MAX_PARAMETERS][MAX_PARAMETERS]; // directions (unit vectors)
		double pt[MAX_PARAMETERS], ptt[MAX_PARAMETERS], xit[MAX_PARAMETERS];
		double del, fp, fptt, t;

		*fret=std::numeric_limits<double>::quiet_NaN();
		if(p==0x0) return(11); if(delta==0x0) return(12);
		if(parNr<1) return(21); if(ftol<=0.0) return(22); if(ftol>=1.0) return(23);
		if((*iterNr)<1) return(24);
	
		// SetUp
		_powellFunc=_fun;
		iterMax=*iterNr; // save the max nr of iterations
		_powell_ncom=parNr;
		// Function value at initial point
		*fret=(*_powellFunc)(p);
		if(!isfinite_impl(*fret)) {*fret=std::numeric_limits<double>::quiet_NaN(); return 1;}
		// Save the initial point
		for(j=0; j<parNr; j++) pt[j]=p[j];
		// Check which parameters are fixed
		for(i=0; i<parNr; i++) if(std::abs(delta[i])<1.0e-20) fixed[i]=1; else fixed[i]=0;

		// Initiate matrix for directions
		for(i=0; i<parNr; i++)
			for(j=0; j<parNr; j++)
				if(i==j) xi[i][j]=delta[i]; else xi[i][j]=0.0;

		// Iterate
		for(*iterNr=1; ; (*iterNr)++) {
			fp=*fret; ibig=0; del=0.0; // largest function decrease

			// In each iteration, loop over all directions in the set
			for(i=0; i<parNr; i++) {
				if(fixed[i]) continue; // do nothing with fixed parameters
				for(j=0; j<parNr; j++) if(fixed[j]) xit[j]=0.0; else xit[j]=xi[j][i];
				fptt=*fret;
				// minimize along direction xit
				_powell_linmin(p, xit, parNr, fret);
				if(std::abs(fptt-(*fret))>del) {del=std::abs(fptt-(*fret)); ibig=i;}
			}

			// Check if done
			if(2.0*std::abs(fp-(*fret)) <= ftol*(std::abs(fp)+std::abs(*fret))) break;
			if((*iterNr)>=iterMax) return 1;

			// Construct the extrapolated point and the average direction moved
			for(j=0; j<parNr; j++) {
				ptt[j]=2.0*p[j]-pt[j]; xit[j]=p[j]-pt[j];
				pt[j]=p[j]; // save the old starting point
			}
			fptt=(*_powellFunc)(ptt);
			if(fptt<fp) {
				t=2.0*(fp-2.0*(*fret)+fptt)*_powell_sqr(fp-(*fret)-del)-del*_powell_sqr(fp-fptt);
				if(t<0.0) {
					_powell_linmin(p, xit, parNr, fret);
					for(j=0; j<parNr; j++) {
						xi[j][ibig]=xi[j][parNr-1]; xi[j][parNr-1]=xit[j];}
				}
			}
		} // next iteration

		//oop
	
		return 0;
	}

	void _powell_linmin(double *p, double *xi, int n, double *fret)
	{
		int i;
		double xx, xmin, fx, fb, fa, bx, ax;

		_powell_ncom=n;
		for(i=0; i<n; i++) {_powell_pcom[i]=p[i]; _powell_xicom[i]=xi[i];}
		ax=0.0; xx=1.0;
		_powell_mnbrak(&ax, &xx, &bx, &fa, &fx, &fb);
		*fret=_powell_brent(ax, xx, bx, 2.0e-4, &xmin);
		for(i=0; i<n; i++) {xi[i]*=xmin; p[i]+=xi[i];}
	}

	double _powell_brent(double ax, double bx, double cx, double tol, double *xmin)
	{
		const int ITMAX = 100; // 2do this should be a paramer
		const double CGOLD = 0.3819660;
		const double ZEPS = 1.0e-10;
		int iter;
		double a, b, d=0.0, etemp, fu, fv, fw, fx, p, q, r;
		double e=0.0, tol1, tol2, u, v, w, x, xm;

		a=(ax<cx ? ax:cx); b=(ax>cx ? ax:cx); x=w=v=bx; fw=fv=fx=_powell_f1dim(x);
		for(iter=0; iter<ITMAX; iter++) {
			xm=0.5*(a+b); tol2=2.0*(tol1=tol*std::abs(x)+ZEPS);
			xm=0.5*(a+b); tol2=2.0*(tol1=tol*std::abs(x)+ZEPS);
			if(std::abs(x-xm)<=(tol2-0.5*(b-a))) {*xmin=x; return(fx);}
			if(std::abs(e)>tol1) {
				r=(x-w)*(fx-fv); q=(x-v)*(fx-fw); p=(x-v)*q-(x-w)*r;
				q=2.0*(q-r); if(q>0.0) p=-p; q=std::abs(q);
				etemp=e; e=d;
				if(std::abs(p)>=std::abs(0.5*q*etemp) || p<=q*(a-x) || p>=q*(b-x))
					d=CGOLD*(e=(x>=xm ? a-x:b-x));
				else {
					d=p/q; u=x+d; if(u-a<tol2 || b-u<tol2) d=_powell_sign(tol1, xm-x);}
			} else {d=CGOLD*(e=(x>=xm ? a-x:b-x));}
			u=(std::abs(d)>=tol1 ? x+d:x+_powell_sign(tol1, d));
			fu=_powell_f1dim(u);
			if(fu<=fx) {
				if(u>=x) a=x; else b=x;
				_powell_shft(&v, &w, &x, &u); _powell_shft(&fv, &fw, &fx, &fu);
			} else {
				if(u<x) a=u; else b=u;
				if(fu<=fw || w==x) {v=w; w=u; fv=fw; fw=fu;}
				else if(fu<=fv || v==x || v==w) {v=u; fv=fu;}
			}
		}
		*xmin=x;
		return(fx);
	}

	double _powell_f1dim(double x)
	{
		int i;
		double f, xt[MAX_PARAMETERS];

		for(i=0; i<_powell_ncom; i++) xt[i]=_powell_pcom[i]+x*_powell_xicom[i];
		f=(*_powellFunc)(xt);
		return(f);
	}

	void _powell_mnbrak(double *ax, double *bx, double *cx, double *fa, double *fb,	double *fc)
	{
		const double GOLD = 1.618034;
		const double GLIMIT = 100.0;
		const double TINY = 1.0e-20;
		double ulim, u, r, q, fu, dum=0.0;

		*fa=_powell_f1dim(*ax); *fb=_powell_f1dim(*bx);
		if(*fb>*fa) {_powell_shft(&dum, ax, bx, &dum); _powell_shft(&dum, fb, fa, &dum);}
		*cx=(*bx)+GOLD*(*bx-*ax); *fc=_powell_f1dim(*cx);
		while((*fb)>(*fc)) {
			r=(*bx-*ax)*(*fb-*fc); q=(*bx-*cx)*(*fb-*fa);
			u=(*bx)-((*bx-*cx)*q-(*bx-*ax)*r)/(2.0*_powell_sign(_powell_fmax(std::abs(q-r),TINY),q-r));
			ulim=(*bx)+GLIMIT*(*cx-*bx);
			if(((*bx)-u)*(u-(*cx)) > 0.0) {
				fu=_powell_f1dim(u);
				if(fu < *fc) {*ax=(*bx); *bx=u; *fa=(*fb); *fb=fu; return;}
				else if(fu > *fb) {*cx=u; *fc=fu; return;}
				u=(*cx)+GOLD*(*cx-*bx);
				fu=_powell_f1dim(u);
			} else if((*cx-u)*(u-ulim) > 0.0) {
				fu=_powell_f1dim(u);
				if(fu < *fc) {
					q=*cx+GOLD*(*cx-*bx); r=_powell_f1dim(u);
					_powell_shft(bx, cx, &u, &q); _powell_shft(fb, fc, &fu, &r);
				}
			} else if((u-ulim)*(ulim-*cx) >= 0.0) {
				u=ulim; fu=_powell_f1dim(u);
			} else {
				u=(*cx)+GOLD*(*cx-*bx);
				fu=_powell_f1dim(u);
			}
			_powell_shft(ax, bx, cx, &u); _powell_shft(fa, fb, fc, &fu);
		}
	}

}
