
float I = 0;
float Kp = 10.0;
float Ki = 1.5;
float Kd = 1.5;
float Ti = (Kp*dt)/Ki;
float Td = (Kd*dt)/Kp;
bool clamp = false;

int getSign(float x)
{
	if (x >= 0)
		return 1;
	else
		return -1;
}

float midpointIntegral(float tilt, float sum)
{
	return sum + tilt*dt;
}

float trapezoidIntegral(float tilt, float prev, float sum)
{
	if (clamp)
		return sum;
		
	return sum + ((tilt + prev)/2.0)*dt;
}

float derivative(float tilt, float prev)
{
	return (tilt - prev)/dt;
}

float controllerPID(float tilt, float prev)
{
	float D = 0, P = 0;

#ifdef P_TERM
	P = tilt;
#endif

#ifdef I_TERM
	I = trapezoidIntegral(tilt, prev, I);
#endif

#ifdef D_TERM
	D = derivative(tilt, prev);
#endif

	Serial.print("clamp = ");
	Serial.println(clamp);
	Serial.print("I = ");
	Serial.println(I);
	Serial.print("D = ");
	Serial.println(D);
	Serial.print("tilt = ");
	Serial.println(tilt);
	
	return Kp*(P + (1.0/Ti)*I + Td*D);
}

float clampPID(float cs, float tilt)
{
	float newCS;
	clamp = false;

	if (abs(cs) > motor.maxSpeed())
		if (cs > 0)
			newCS =  motor.maxSpeed();
		else
			newCS =  -1.0*motor.maxSpeed();

	else
		newCS = cs;

	if (newCS == cs)
		return newCS;
	
	if (getSign(cs) != getSign(tilt))
		return newCS;

	clamp = true;

	return newCS;
}
