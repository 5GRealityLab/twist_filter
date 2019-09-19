float64 linear_vel_max      # Max linear velocity
float64 linear_acc_max      # Max linear acceleration
float64 angular_vel_max     # Max angular velocity
float64 angular_acc_max     # Max angular acceleration

int32 num_samples           # Sample size
float64[] weights           # Array of weights (optional)

int32 num_out_samples       # Feedback sample size for IIR filter
float64[] out_weights       # Feedback weights for IIR filter