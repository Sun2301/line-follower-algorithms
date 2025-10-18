
// SMC state variables - we hhave to declare these globally in our main code

float smc_lastError = 0;
float smc_errorDotFiltered = 0;

// SMC parameters - tune these for our robot
float smc_lambda = 1.0;      // Sliding surface slope
float smc_K_max = 50.0;      // Maximum control gain
float smc_delta = 0.1;       // Boundary layer thickness
float smc_filterAlpha = 0.7; // Derivative filter coefficient

/**
 * Sliding Mode Control function
 * 
 * @param error - Current position error (negative = left, positive = right)
 * @param dt - Time step in seconds (e.g., 0.01 for 100Hz)
 * @return Control output (negative = turn left, positive = turn right)
 */
float SMC_control(float error, float dt) {
  // Calculate error derivative
  float errorDot = (error - smc_lastError) / dt;
  
  // Filter the derivative to reduce noise
  smc_errorDotFiltered = smc_filterAlpha * smc_errorDotFiltered + 
                         (1 - smc_filterAlpha) * errorDot;
  
  // Sliding surface: s = e + λ*ė
  float slidingSurface = error + smc_lambda * smc_errorDotFiltered;
  
  // SMC control law: u = -K*s/(|s| + δ)
  float controlOutput = -smc_K_max * slidingSurface / 
                        (abs(slidingSurface) + smc_delta);
  
  // Update state for next iteration
  smc_lastError = error;
  
  return controlOutput;
}

/**
 * Reset SMC state - to be called when line is lost or starting fresh
 */
void SMC_reset() {
  smc_lastError = 0;
  smc_errorDotFiltered = 0;
}