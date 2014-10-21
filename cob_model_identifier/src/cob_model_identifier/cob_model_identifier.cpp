


// Bogacki-Shampine method 
void integrate(double &t, double &y, double &h) { 
	double k1, k2, k3, k4, y1, y2; // f(t, y) is our derivative 
	k1 = f(t, y) k2 = f(t + h * 0.50, y + h * 0.50 * k1); 
	k3 = f(t + h * 0.75, y + h * 0.75 * k2);
	
	// 3rd order accurate solution 
	y1 = y + h * (2.0/9.0 * k1 + 1.0/3.0 * k2 + 4.0/9.0 * k3); k4 = f(t + h, y1);
	
	// 2nd order accurate solution 
	y2 = y + h1 * (7.0/24.0 * k1 + 1.0/4.0 * k2 + 1.0/3.0 * k3 + 1.0/8.0 * k4); y = y2; t += h;
	 
	// Define atol (absolute tolerance) and rtol (relative tolerance) somewhere 
	// We use the difference between the two solutions as an approximation of the error, scaled by our desired absolute and relative tolerance 
	// Both atol and rtol should be non-zero. 
	double scale = atol + max(fabs(y1), fabs(y2)) * rtol;
	double error = max(fabs(y2 - y1), 2E-10) / scale; h = h * pow(error, -1.0/3.0); 
	}

