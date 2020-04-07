#include <tf/LinearMath/Quaternion.h>
#include <tf/tf.h>

namespace madgwickahrs{

class MadgwickAHRS{
private:
	float SamplePeriod;
	float Beta;
	float Quaternion[4];

public:

	MadgwickAHRS(float samplePeriod, float beta);
	virtual ~MadgwickAHRS();

	void Update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
	void Update(float gx, float gy, float gz, float ax, float ay, float az);

	float getBeta() const {
		return Beta;
	}

	void setBeta(float beta) {
		Beta = beta;
	}

	float getSamplePeriod() const {
		return SamplePeriod;
	}

	void setSamplePeriod(float samplePeriod) {
		this->SamplePeriod = samplePeriod;
	}

	float getQ0(){
		return Quaternion[0];
	}
	float getQ1(){
		return Quaternion[1];
	}
	float getQ2(){
		return Quaternion[2];
	}
	float getQ3(){
		return Quaternion[3];
	}

	void setQuaternion(float q0, float q1, float q2, float q3){
		Quaternion[0] = q0;
		Quaternion[1] = q1;
		Quaternion[2] = q2;
		Quaternion[3] = q3;
	}
};

}
