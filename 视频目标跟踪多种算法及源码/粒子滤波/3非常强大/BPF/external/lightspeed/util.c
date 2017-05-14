/* C implementations of useful functions.
 * Written by Tom Minka (unless otherwise noted).
 */

#define _USE_MATH_DEFINES 1
#include <math.h>
#include <stdlib.h>
#include <float.h>
#include "util.h"
#include "mex.h"

#ifdef _MSC_VER
#define finite _finite
#define isnan _isnan
#endif

#ifdef	 __USE_ISOC99
/* INFINITY and NAN are defined by the ISO C99 standard */
#else
double my_infinity(void) {
  double zero = 0;
  return 1.0/zero;
}
double my_nan(void) {
  double zero = 0;
  return zero/zero;
}
#define INFINITY my_infinity()
#define NAN my_nan()
#endif

/*
*  Generates a uniformly distributed r.v. between 0 and 1.
*  Kris Popat  6/85
*  Ref: Applied Statistics, 1982 vol 31 no 2 pp 188-190
*  Based on FORTRAN routine by H. Malvar.
*/

static long ix = 101;
static long iy = 1001;
static long iz = 10001;

double Rand(void)
{
  static float u;
  
  ix = 171*(ix % 177)-2*(ix/177);
  iy = 172*(iy % 176)-2*(iy/176);
  iz = 170*(iz % 178)-2*(iz/178);
  
  if (ix<0) ix = ix + 30269;
  if (iy<0) iy = iy + 30307;
  if (iz<0) iz = iz + 30323;
  
  u = ((float) ix)/30269.0 +
                ((float) iy)/30307.0 + ((float) iz)/30323.0;
  u -= (float)(int)u;
  return(u);
}

/* Returns a sample from Gamma(a, 1).
 * For Gamma(a,b), scale the result by b.
 */
double GammaRand(double a)
{
#if 1
  /* Algorithm from BUGS */
  double c1, c2, c3, c4, c5, c6;
  if(isnan(a)) return a;
  if(a < DBL_EPSILON) return 0;
  /* If a == 1, then gamma is exponential. */
  /* It is important that Rand() is never zero. */
  if(fabs(a - 1) < DBL_EPSILON) return -log(Rand());
  if(a < 1) {
    c6 = exp(log(Rand())/a);
    a = a + 1;
  }
  else c6 = 1;
  c1 = a - 1;
  c2 = (a - 1/(6*a))/c1;
  c3 = 2/c1;
  c4 = c3 + 2;
  c5 = 1/sqrt(a);
  c6 *= c1;
  while(1) {
    double u1, u2, w;
    /* loop until u1 is valid */
    do {
      u1 = Rand();
      u2 = Rand();
      if(a > 2.5) u1 = u2 + c5*(1 - 1.86*u1);
    } while((u1 <= 0) || (u1 >= 1));
    w = c2*u2/u1;
    if((c3*u1 + w + 1/w <= c4) ||
       (c3*log(u1) - log(w) + w < 1))
      return c6*w;
  }
  /* never get here */
  return -1;
#else
  /* Algorithm:
   * G. Marsaglia and W.W. Tsang, A simple method for generating gamma
   * variables, ACM Transactions on Mathematical Software, Vol. 26, No. 3,
   * Pages 363-372, September, 2000.
   * http://portal.acm.org/citation.cfm?id=358414
   */
  double boost, d, c, v;
  if(a < 1) {
    /* boost using Marsaglia's (1961) method: gam(a) = gam(a+1)*U^(1/a) */
    boost = exp(log(Rand())/a);
    a++;
  } 
  else boost = 1;
  d = a-1.0/3; c = 1.0/sqrt(9*d);
  while(1) {
    double x;
    do {
      x = RandN();
      v = 1+c*x;
    } while(v <= 0);
    v = v*v*v;
    x = x*x;
    double u = Rand();
    if((u < 1-.0331*x*x) || 
       (log(u) < 0.5*x + d*(1-v+log(v)))) break;
  }
  return( boost*d*v );
#endif
}

/* Returns a sample from Beta(a,b) */
double BetaRand(double a, double b)
{
  double g = GammaRand(a);
  return g/(g + GammaRand(b));
}

/* Very fast binomial sampler. 
 * Returns the number of successes out of n trials, with success probability p.
 */
int BinoRand(double p, int n)
{
  int r = 0;
  if(isnan(p)) return 0;
  if(p < DBL_EPSILON) return 0;
  if(p >= 1-DBL_EPSILON) return n;
  if((p > 0.5) && (n < 15)) {
    /* Coin flip method. This takes O(n) time. */
    int i;
    for(i=0;i<n;i++) {
      if(Rand() < p) r++;
    }
    return r;
  }
  if(n*p < 10) {
    /* Waiting time method.  This takes O(np) time. */
    double q = -log(1-p), e = -log(Rand()), s;
    r = n;
    for(s = e/r; s <= q; s += e/r) {
      r--;
      if(r == 0) break;
      e = -log(Rand());
    }
    r = n-r;
    return r;
  }
  if (1) {
    /* Recursive method.  This makes O(log(log(n))) recursive calls. */
    int i = floor(p*(n+1));
    double b = BetaRand(i, n+1-i);
    if(b <= p) r = i + BinoRand((p-b)/(1-b), n-i);
    else r = i - BinoRand((b-p)/b, i-1);
    return r;
  }
}

/****************************************************************************/

double logSum(double a, double b)
{
  /* make a the larger number */
  if(a < b) {
    double t = a; a = b; b = t;
  }
  /* log(exp(a) + exp(b)) = a + log(1 + exp(b-a)) */
  if(!finite(b)) return a;
  return a + log(1 + exp(b-a));
}

#define CACHE_SIZE 200

/* Requires: n >= 0 */
double pochhammer(double x, int n)
{
  static double cache_x = -1;
  static double cache_v[CACHE_SIZE];
  static int max_cached;
  double result;
  int i;
  /* the maximum n for which we have a cached value */
  if(n == 0) return 0;
  if(n > CACHE_SIZE) {
    if(x >= 1.e4*n) {
      return log(x) + (n-1)*log(x+n/2);
    }
    return gammaln(x+n) - gammaln(x);
  }
  if(x != cache_x) {
    max_cached = 1;
    cache_v[0] = log(x);
    cache_x = x;
  }
  if(n <= max_cached) return cache_v[n-1];
  result = cache_v[max_cached-1];
  x = x + max_cached-1;
  for(i=max_cached;i<n;i++) {
    x = x + 1;
    result += log(x);
    cache_v[i] = result;
  }
  max_cached = n;
  return result;
}

/* Requires: n >= 0 */
double slow_pochhammer(double x, int n)
{
  double result;
  if(n == 0) return 0;
  if(n <= 20) {
    int i;
    double xi = x;
    /* this assumes x is not too large */
    result = xi;
    for(i=n-1; i > 0; i--) {
      xi = xi + 1;
      result *= xi;
    }
    result = log(result);
  }
  else if(x >= 1.e4*n) {
    result = log(x) + (n-1)*log(x+n/2);
  }
  else result = gammaln(x+n) - gammaln(x);
  return result;
}

double di_pochhammer(double x, int n)
{
  static double cache_x = -1;
  static double cache_v[CACHE_SIZE];
  static int max_cached;
  double result;
  int i;
  /* the maximum n for which we have a cached value */
  if(n == 0) return 0;
  if(n > CACHE_SIZE) {
    return digamma(x+n) - digamma(x);
  }
  if(x != cache_x) {
    max_cached = 1;
    cache_v[0] = 1/x;
    cache_x = x;
  }
  if(n <= max_cached) return cache_v[n-1];
  result = cache_v[max_cached-1];
  x = x + max_cached-1;
  for(i=max_cached;i<n;i++) {
    x = x + 1;
    result += 1/x;
    cache_v[i] = result;
  }
  max_cached = n;
  return result;
}

double slow_di_pochhammer(double x, int n)
{
  double result;
  if(n == 0) return 0;
  if(n <= 20) {
    int i;
    double xi = x;
    result = 1/xi;
    for(i=n-1; i > 0; i--) {
      xi = xi + 1;
      result += 1/xi;
    }
  }
  else result = digamma(x+n) - digamma(x);
  return result;
}

double tri_pochhammer(double x, int n)
{
  static double cache_x = -1;
  static double cache_v[CACHE_SIZE];
  static int max_cached;
  double result;
  int i;
  /* the maximum n for which we have a cached value */
  if(n == 0) return 0;
  if(n > CACHE_SIZE) {
    return trigamma(x+n) - trigamma(x);
  }
  if(x != cache_x) {
    max_cached = 1;
    cache_v[0] = -1/(x*x);
    cache_x = x;
  }
  if(n <= max_cached) return cache_v[n-1];
  result = cache_v[max_cached-1];
  x = x + max_cached-1;
  for(i=max_cached;i<n;i++) {
    x = x + 1;
    result -= 1/(x*x);
    cache_v[i] = result;
  }
  max_cached = n;
  return result;
}

double slow_tri_pochhammer(double x, int n)
{
  double result;
  if(n == 0) return 0;
  if(n <= 20) {
    result = -1/(x*x);
    n--;
    while(n > 0) {
      x = x + 1;
      result -= 1/(x*x);
      n--;
    }
    return result;
  }
  return trigamma(x+n) - trigamma(x);
}

/* Logarithm of multivariate Gamma function, defined as
 * Gamma_d(x) = pi^(d*(d-1)/4)*prod_(i=1..d) Gamma(x + (1-i)/2)
 * http://en.wikipedia.org/wiki/Multivariate_gamma_function
 */
double gammaln2(double x, double d)
{
  #define M_lnPI 1.14472988584940
  double r = d*(d-1)/4*M_lnPI;
  int i;
  for(i=0; i<d; i++) r += gammaln(x + (1-i)/2);
  return r;
}

/* Logarithm of the gamma function.
   Returns NaN for negative arguments, even though log(gamma(x)) may
   actually be defined.

   References:

   1) W. J. Cody and K. E. Hillstrom, 'Chebyshev Approximations for
      the Natural Logarithm of the Gamma Function,' Math. Comp. 21,
      1967, pp. 198-203.

   2) K. E. Hillstrom, ANL/AMD Program ANLC366S, DGAMMA/DLGAMA, May,
      1969.
 
   3) Hart, Et. Al., Computer Approximations, Wiley and sons, New
      York, 1968.

   From matlab/gammaln.m
*/
double gammaln(double x)
{
  double result, y, xnum, xden;
  int i;
  static double d1 = -5.772156649015328605195174e-1;
  static double p1[] = { 
    4.945235359296727046734888e0, 2.018112620856775083915565e2, 
    2.290838373831346393026739e3, 1.131967205903380828685045e4, 
    2.855724635671635335736389e4, 3.848496228443793359990269e4, 
    2.637748787624195437963534e4, 7.225813979700288197698961e3 
  };
  static double q1[] = {
    6.748212550303777196073036e1, 1.113332393857199323513008e3, 
    7.738757056935398733233834e3, 2.763987074403340708898585e4, 
    5.499310206226157329794414e4, 6.161122180066002127833352e4, 
    3.635127591501940507276287e4, 8.785536302431013170870835e3
  };
  static double d2 = 4.227843350984671393993777e-1;
  static double p2[] = {
    4.974607845568932035012064e0, 5.424138599891070494101986e2, 
    1.550693864978364947665077e4, 1.847932904445632425417223e5, 
    1.088204769468828767498470e6, 3.338152967987029735917223e6, 
    5.106661678927352456275255e6, 3.074109054850539556250927e6
  };
  static double q2[] = {
    1.830328399370592604055942e2, 7.765049321445005871323047e3, 
    1.331903827966074194402448e5, 1.136705821321969608938755e6, 
    5.267964117437946917577538e6, 1.346701454311101692290052e7, 
    1.782736530353274213975932e7, 9.533095591844353613395747e6
  };
  static double d4 = 1.791759469228055000094023e0;
  static double p4[] = {
    1.474502166059939948905062e4, 2.426813369486704502836312e6, 
    1.214755574045093227939592e8, 2.663432449630976949898078e9, 
    2.940378956634553899906876e10, 1.702665737765398868392998e11, 
    4.926125793377430887588120e11, 5.606251856223951465078242e11
  };
  static double q4[] = {
    2.690530175870899333379843e3, 6.393885654300092398984238e5, 
    4.135599930241388052042842e7, 1.120872109616147941376570e9, 
    1.488613728678813811542398e10, 1.016803586272438228077304e11, 
    3.417476345507377132798597e11, 4.463158187419713286462081e11
  };
  static double c[] = {
    -1.910444077728e-03, 8.4171387781295e-04, 
    -5.952379913043012e-04, 7.93650793500350248e-04, 
    -2.777777777777681622553e-03, 8.333333333333333331554247e-02, 
    5.7083835261e-03
  };
  static double a = 0.6796875;

  if((x <= 0.5) || ((x > a) && (x <= 1.5))) {
    if(x <= 0.5) {
      result = -log(x);
      /*  Test whether X < machine epsilon. */
      if(x+1 == 1) {
	return result;
      }
    }
    else {
      result = 0;
      x = (x - 0.5) - 0.5;
    }
    xnum = 0;
    xden = 1;
    for(i=0;i<8;i++) {
      xnum = xnum * x + p1[i];
      xden = xden * x + q1[i];
    }
    result += x*(d1 + x*(xnum/xden));
  }
  else if((x <= a) || ((x > 1.5) && (x <= 4))) {
    if(x <= a) {
      result = -log(x);
      x = (x - 0.5) - 0.5;
    }
    else {
      result = 0;
      x -= 2;
    }
    xnum = 0;
    xden = 1;
    for(i=0;i<8;i++) {
      xnum = xnum * x + p2[i];
      xden = xden * x + q2[i];
    }
    result += x*(d2 + x*(xnum/xden));
  }
  else if(x <= 12) {
    x -= 4;
    xnum = 0;
    xden = -1;
    for(i=0;i<8;i++) {
      xnum = xnum * x + p4[i];
      xden = xden * x + q4[i];
    }
    result = d4 + x*(xnum/xden);
  }
  /*  X > 12  */
  else {
    y = log(x);
    /* result = x*(y - 1) - y*0.5 + .9189385332046727417803297; */
    /* want to preserve infinity */
    result = (x - 0.5)*(y - 1) + .4189385332046727417803297;
    x = 1/x;
    y = x*x;
    xnum = c[6];
    for(i=0;i<6;i++) {
      xnum = xnum * y + c[i];
    }
    xnum *= x;
    result += xnum;
  }
  return result;
}

/* The digamma function is the derivative of gammaln.

   Reference:
    J Bernardo,
    Psi ( Digamma ) Function,
    Algorithm AS 103,
    Applied Statistics,
    Volume 25, Number 3, pages 315-317, 1976.

    From http://www.psc.edu/~burkardt/src/dirichlet/dirichlet.f
    (with modifications for negative numbers and extra precision)
*/
double digamma(double x)
{
  double neginf = -INFINITY;
  static const double c = 12,
    d1 = -0.57721566490153286,
    d2 = 1.6449340668482264365, /* pi^2/6 */
    s = 1e-6,
    s3 = 1./12,
    s4 = 1./120,
    s5 = 1./252,
    s6 = 1./240,
    s7 = 1./132,
    s8 = 691/32760,
    s9 = 1/12,
    s10 = 3617/8160;
  double result;
#if 0
  static double cache_x = 0;
  static int hits = 0, total = 0;
  total++;
  if(x == cache_x) {
    hits++;
  }
  if(total % 1000 == 1) {
    printf("hits = %d, total = %d, hits/total = %g\n", hits, total, 
	   ((double)hits)/total);
  }
  cache_x = x;
#endif
  /* Illegal arguments */
  if((x == neginf) || isnan(x)) {
    return NAN;
  }
  /* Singularities */
  if((x <= 0) && (floor(x) == x)) {
    return neginf;
  }
  /* Negative values */
  /* Use the reflection formula (Jeffrey 11.1.6):
   * digamma(-x) = digamma(x+1) + pi*cot(pi*x)
   *
   * This is related to the identity
   * digamma(-x) = digamma(x+1) - digamma(z) + digamma(1-z)
   * where z is the fractional part of x
   * For example:
   * digamma(-3.1) = 1/3.1 + 1/2.1 + 1/1.1 + 1/0.1 + digamma(1-0.1)
   *               = digamma(4.1) - digamma(0.1) + digamma(1-0.1)
   * Then we use
   * digamma(1-z) - digamma(z) = pi*cot(pi*z)
   */
  if(x < 0) {
    return digamma(1-x) + M_PI/tan(-M_PI*x);
  }
  /* Use Taylor series if argument <= S */
  if(x <= s) return d1 - 1/x + d2*x;
  /* Reduce to digamma(X + N) where (X + N) >= C */
  result = 0;
  while(x < c) {
    result -= 1/x;
    x++;
  }
  /* Use de Moivre's expansion if argument >= C */
  /* This expansion can be computed in Maple via asympt(Psi(x),x) */
  if(x >= c) {
    double r = 1/x, t;
    result += log(x) - 0.5*r;
    r *= r;
#if 0
    result -= r * (s3 - r * (s4 - r * (s5 - r * (s6 - r * s7))));
#else
    /* this version for lame compilers */
    t = (s5 - r * (s6 - r * s7));
    result -= r * (s3 - r * (s4 - r * t));
#endif
  }
  return result;
}

/* The trigamma function is the derivative of the digamma function.

   Reference:

    B Schneider,
    Trigamma Function,
    Algorithm AS 121,
    Applied Statistics, 
    Volume 27, Number 1, page 97-99, 1978.

    From http://www.psc.edu/~burkardt/src/dirichlet/dirichlet.f
    (with modification for negative arguments and extra precision)
*/
double trigamma(double x)
{
  double neginf = -INFINITY,
    small = 1e-4,
    large = 8,
    c = 1.6449340668482264365, /* pi^2/6 = Zeta(2) */
    c1 = -2.404113806319188570799476,  /* -2 Zeta(3) */
    b2 =  1./6,
    b4 = -1./30,
    b6 =  1./42,
    b8 = -1./30,
    b10 = 5./66;
  double result;
  /* Illegal arguments */
  if((x == neginf) || isnan(x)) {
    return NAN;
  }
  /* Singularities */
  if((x <= 0) && (floor(x) == x)) {
    return neginf;
  }
  /* Negative values */
  /* Use the derivative of the digamma reflection formula:
   * -trigamma(-x) = trigamma(x+1) - (pi*csc(pi*x))^2
   */
  if(x < 0) {
    result = M_PI/sin(-M_PI*x);
    return -trigamma(1-x) + result*result;
  }
  /* Use Taylor series if argument <= small */
  if(x <= small) {
    return 1/(x*x) + c + c1*x;
  }
  result = 0;
  /* Reduce to trigamma(x+n) where ( X + N ) >= B */
  while(x < large) {
    result += 1/(x*x);
    x++;
  }
  /* Apply asymptotic formula when X >= B */
  /* This expansion can be computed in Maple via asympt(Psi(1,x),x) */
  if(x >= large) {
    double r = 1/(x*x), t;
#if 0
    result += 0.5*r + (1 + r*(b2 + r*(b4 + r*(b6 + r*(b8 + r*b10)))))/x;
#else
    t = (b4 + r*(b6 + r*(b8 + r*b10)));
    result += 0.5*r + (1 + r*(b2 + r*t))/x;
#endif
  }
  return result;
}

unsigned *ismember_sorted(double *a, unsigned a_len, double *s, unsigned s_len)
{
  /* returns a vector tf where tf[i] = 1 if a[i] is in s. */
  unsigned *tf = mxCalloc(a_len,sizeof(unsigned));
  unsigned i,j=0;
  if(j == s_len) return tf;
  for(i=0;i<a_len;i++) {
    while(s[j] < a[i]) {
      j++;
      if(j == s_len) return tf;
    }
    if(s[j] == a[i]) tf[i] = 1;
  }
  return tf;
}


