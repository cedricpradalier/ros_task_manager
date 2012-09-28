#ifndef PID_CLASS_H
#define PID_CLASS_H

#include <math.h>
#include <string>


class PID
{
	protected:
		std::string name;
		double I;
		bool autodt;
		double dt, last_t;
		double Kp,Ki,Kd;
		double e, last_e;
		double Imax,Omax,Emax;
		bool angle;
	public:
		PID(const std::string & nm) : name(nm), I(0), autodt(true), dt(0), last_t(-1), 
			Kp(0), Ki(0), Kd(0), e(0), last_e(0),
			Imax(-1), Omax(-1), Emax(-1), angle(false) { }

		PID(const std::string & nm, double kp, double ki, double kd) 
			: name(nm), I(0), autodt(true), dt(0), last_t(-1), 
			Kp(kp), Ki(ki), Kd(kd), e(0), last_e(0),
			Imax(-1), Omax(-1), Emax(-1), angle(false) { }

		PID(const std::string & nm, double _dt, double kp, double ki, double kd) 
			: name(nm), I(0), autodt(false), dt(_dt), last_t(-1),
			Kp(kp), Ki(ki), Kd(kd), e(0), last_e(0),
			Imax(-1), Omax(-1), Emax(-1), angle(false) { }

		~PID() {}

		void setGains(double kp,double ki,double kd) {
			Kp=kp;
			Ki=ki;
			Kd=kd;
		}
		void setKp(double kp) {Kp=kp;}
		void setKi(double ki) {Ki=ki;}
		void setKd(double kd) {Kd=kd;}

		void setImax(double mx) {Imax=mx;}
		void setOmax(double mx) {Omax=mx;}
		void setEmax(double mx) {Emax=mx;}
		void reset() {I=0.0;}

		void setAngle(bool a) {angle=a;}

		double operator()(double ystar, double y);
		double operator()(double e);

		void print() const {
			printf("PID '%s': %.2g/%.2g/%.2g : I %.2f\n",
					name.c_str(),Kp,Ki,Kd,I);
		}

};


#endif // PID_CLASS_H
