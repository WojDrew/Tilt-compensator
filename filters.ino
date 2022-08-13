

/*
 * return tilt angle in degrees
 */
float complementaryFilter(float accX, float accZ,
			  float vel, float angle)
{
	float newAngle, accAngle;

	accAngle = getAccAngle(accX, accZ);

	newAngle = getGyroAngle(vel, angle);

	newAngle = newAngle * 0.98 + accAngle * 0.02;

	return newAngle;
}

float getGyroAngle(float vel, float angle)
{
	return angle - vel * dt;
}

float getAccAngle(float accX, float accZ)
{
	 return atan2f(accX, accZ) * 180 / M_PI;
}
