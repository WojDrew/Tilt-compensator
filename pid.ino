
float C = 0;
float Kp = 10.0;
float Ki = 1.5;
float Kd = 1.5;
float Ti = (Kp*dt)/Ki;
float Td = (Kd*dt)/Kp;

float midpointIntegral(float tilt, float sum)
{
	return sum + tilt*dt;
}

float trapezoidIntegral(float tilt, float prev, float sum)
{
	return sum + ((tilt + prev)/2.0)*dt;
}

float derivative(float tilt, float prev)
{
	return (tilt - prev)/dt;
}

float controllerPID(float tilt, float prev)
{
	float C = midpointIntegral(tilt, C);
	float D = derivative(tilt, prev);
	
	return Kp*(tilt + (1.0/Ti)*C + Td*D);
}
