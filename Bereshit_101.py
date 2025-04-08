import math
from Moon import Moon
import matplotlib.pyplot as plt 

# Constants (Based on Bereshit's specifications)
WEIGHT_EMP = 165       # kg
WEIGHT_FULE = 420      # kg
WEIGHT_FULL = WEIGHT_EMP + WEIGHT_FULE  # kg
MAIN_ENG_F = 430       # N
SECOND_ENG_F = 25      # N
MAIN_BURN = 0.15       # kg/sec (assuming fuel density = 1 kg/liter)
SECOND_BURN = 0.009    # kg/sec
ALL_BURN = MAIN_BURN + 8 * SECOND_BURN # Assuming 8 secondary thrusters?

# Helper functions
def acc(weight, main, seconds):
    t = 0
    if main:
        t += MAIN_ENG_F
    t += seconds * SECOND_ENG_F
    return t / weight

def acc_max(weight):
    max_thrust = MAIN_ENG_F + 8 * SECOND_ENG_F # Total max force
    return max_thrust / weight

# --- PID Controller Function ---
def pid_controller(error, error_sum, last_error, last_derivative, dt, kp, ki, kd, min_out=-float('inf'), max_out=float('inf'), min_sum=-float('inf'), max_sum=float('inf')):
    current_error_sum = error_sum + error * dt
    current_error_sum = max(min_sum, min(max_sum, current_error_sum))
    
    derivative = (error - last_error) / dt if dt > 0 else 0
    # Apply simple low-pass filter to derivative term to reduce noise
    filtered_derivative = 0.7 * derivative + 0.3 * last_derivative
    
    control_output = kp * error + ki * current_error_sum + kd * filtered_derivative
    clamped_control_output = max(min_out, min(max_out, control_output))
    
    return clamped_control_output, current_error_sum, error, filtered_derivative

# --- Target Speed Functions with Linear Interpolation ---

def get_vs_target(current_altitude):
    # Define altitude breakpoints and corresponding target speeds
    # Significantly steeper transitions with more dramatic gradient changes
    alt_breakpoints = [2, 5, 20, 100, 300, 800, 2000, 5000,7000, 10000, float('inf')]
    vs_targets =     [0, 1, 3, 5, 7, 10, 15, 20, 25, 28, 30]
    
    # Find the appropriate interval
    for i in range(len(alt_breakpoints) - 1):
        if current_altitude <= alt_breakpoints[i+1]:
            # Linear interpolation formula: y = y1 + (x - x1) * (y2 - y1) / (x2 - x1)
            alt_low, alt_high = alt_breakpoints[i], alt_breakpoints[i+1]
            vs_low, vs_high = vs_targets[i], vs_targets[i+1]
            
            # Handle the case where alt_high is infinity
            if alt_high == float('inf'):
                return vs_high
                
            # Perform linear interpolation
            vs_target = vs_low + (current_altitude - alt_low) * (vs_high - vs_low) / (alt_high - alt_low)
            return vs_target
    
    # Fallback return (should not reach here due to infinity in breakpoints)
    return vs_targets[-1]

def get_hs_target(current_altitude):
    # Define altitude breakpoints and corresponding target speeds
    # Adjusted for steeper transitions at critical altitudes
    alt_breakpoints = [2, 5, 20, 50,100, 300, 800, 2000, 5000, 10000, float('inf')]
    hs_targets =     [0, 1, 3, 8,20, 50, 150, 350, 600, 900, 900]
    
    # Find the appropriate interval
    for i in range(len(alt_breakpoints) - 1):
        if current_altitude <= alt_breakpoints[i+1]:
            # Linear interpolation formula: y = y1 + (x - x1) * (y2 - y1) / (x2 - x1)
            alt_low, alt_high = alt_breakpoints[i], alt_breakpoints[i+1]
            hs_low, hs_high = hs_targets[i], hs_targets[i+1]
            
            # Handle the case where alt_high is infinity
            if alt_high == float('inf'):
                return hs_high
                
            # Perform linear interpolation
            hs_target = hs_low + (current_altitude - alt_low) * (hs_high - hs_low) / (alt_high - alt_low)
            return hs_target
    
    # Fallback return (should not reach here due to infinity in breakpoints)
    return hs_targets[-1]

# --- Simulation ---
def simulate():
    print("Simulating Bereshit's Landing with Thrust Vector PID Control:")
    # Initial conditions
    vs = 24.8 # !! NOTE: positive vertical speed means downwards
    hs = 932
    dist = 181 * 1000
    ang = 58.3 
    alt = 13748
    time = 0
    dt = 0.5
    fuel = 121
    weight = WEIGHT_EMP + fuel

    # PID State Variables 
    vs_kp = 0.05
    vs_ki = 0.002 
    vs_kd = 0.3  
    vs_error_sum = 0
    vs_last_error = 0
    vs_last_derivative = 0
    # Anti-windup limits
    vs_min_sum = -20
    vs_max_sum = 20

    # Horizontal speed control parameters
    hs_kp = 0.035 #increasing --> causing oscillation in vs
    hs_ki = 0.002
    hs_kd = 0.2
    hs_error_sum = 0
    hs_last_error = 0
    hs_last_derivative = 0
    hs_min_sum = -10
    hs_max_sum = 10

    # Angle and throttle control settings
    max_angle_change = 2.0
    max_throttle_change = 0.2  # Limit throttle changes to prevent abrupt changes
    current_angle_deg = ang
    last_NN = 0.5  # Initial throttle setting

    # --- Data History for Plotting ---
    time_history = []
    alt_history = []
    vs_history = []
    vs_target_history = []
    hs_history = []
    hs_target_history = []
    ang_history = []
    nn_history = []
    # ----------------------------------

    print("time, alt, vs, hs, ang, fuel, weight, thrust_N, NN")

    # Main simulation loop
    while alt > 1: 
        # Store history data
        time_history.append(time)
        alt_history.append(alt)
        vs_history.append(vs)
        hs_history.append(hs)
        ang_history.append(current_angle_deg)
        
        # Calculate target speeds using linear interpolation
        vs_target = get_vs_target(alt)
        hs_target = get_hs_target(alt)
        vs_target_history.append(vs_target)
        hs_target_history.append(hs_target)
        nn_history.append(0)  # Will update later in the loop

        # Adjust PID parameters based on altitude for landing phase
        if alt < 100:
            vs_kp = 0.01
            vs_ki = 0.002
            vs_kd = 0.4
            hs_kp = 0.05
            hs_ki = 0.08
            hs_kd = 0.02
    



        # Calculate errors
        vs_error = vs - vs_target
        hs_error = hs - hs_target

        # Get lunar gravity
        vacc_downward = Moon.get_acc(hs)


        # Run PID Controllers with improved derivative filtering
        ay_net_desired, vs_error_sum, vs_last_error, vs_last_derivative = pid_controller(
            vs_error, vs_error_sum, vs_last_error, vs_last_derivative, dt,
            vs_kp, vs_ki, vs_kd,
            min_sum=vs_min_sum, max_sum=vs_max_sum
        )
        
        ay_thrust_desired = ay_net_desired + vacc_downward

        ax_thrust_desired, hs_error_sum, hs_last_error, hs_last_derivative = pid_controller(
            hs_error, hs_error_sum, hs_last_error, hs_last_derivative, dt,
            hs_kp, hs_ki, hs_kd,
            min_sum=hs_min_sum, max_sum=hs_max_sum
        )

        # Calculate required forces and thrust
        fx_thrust_needed = weight * ax_thrust_desired
        fy_thrust_needed = weight * ay_thrust_desired
        fy_thrust_needed = max(0, fy_thrust_needed)

        thrust_needed_N = math.sqrt(fx_thrust_needed**2 + fy_thrust_needed**2)
        angle_needed_rad = math.atan2(fx_thrust_needed, fy_thrust_needed)
        angle_needed_deg = math.degrees(angle_needed_rad)

        # Constrain angle within limits
        target_angle_deg = angle_needed_deg
        target_angle_deg = max(-60, min(60, target_angle_deg))

        # Rate limiting for angle changes
        angle_diff = target_angle_deg - current_angle_deg
        angle_change_this_step = max(-max_angle_change * dt, min(max_angle_change * dt, angle_diff))
        current_angle_deg += angle_change_this_step
        
        # Additional angle constraints near landing
        if alt <= 5:
            current_angle_deg = max(3, min(3, current_angle_deg))

        max_thrust_N = acc_max(weight) * weight
        # Base throttle for hover + PID adjustment
        NN_desired = thrust_needed_N / max_thrust_N if max_thrust_N > 0 else 0
        
        # Apply rate limiting to throttle changes
        NN_change = NN_desired - last_NN
        NN_change = max(-max_throttle_change * dt, min(max_throttle_change * dt, NN_change))
        NN = last_NN + NN_change
        NN = max(0.0, min(1.0, NN))
        last_NN = NN  # Store for next iteration
        
        actual_thrust_N = NN * max_thrust_N
        nn_history[-1] = NN  # Update NN value in history

        # --- Fuel consumption --- (Do not touch)
        dw = dt * ALL_BURN * NN
        if fuel <= 0:
            actual_thrust_N = 0
            NN = 0
            fuel = 0
            nn_history[-1] = NN
        else:
            if fuel < dw:
                dw = fuel
            fuel -= dw
            weight = WEIGHT_EMP + fuel

        # --- Physics Update --- (Do not touch)
        acc_val = actual_thrust_N / weight if weight > 0 else 0
        ang_rad = math.radians(current_angle_deg)
        h_acc_thrust = math.sin(ang_rad) * acc_val
        v_acc_thrust = math.cos(ang_rad) * acc_val
        v_acc_net = v_acc_thrust - vacc_downward

        hs_prev = hs
        hs -= h_acc_thrust * dt
        hs = max(0, hs)

        vs -= v_acc_net * dt

        dist -= ((hs + hs_prev) / 2.0) * dt
        alt_prev = alt
        alt -= vs * dt

        if alt <= 0:
            # If alt crosses zero, try to estimate landing state more accurately
            frac = alt_prev / (alt_prev - alt) if (alt_prev - alt) != 0 else 1.0
            time += dt * frac
            alt = 0  # Landed
            vs = vs  # Use final calculated vs
            hs = hs  # Use final calculated hs
            dist -= hs * dt * (1-frac)  # Adjust dist for partial step
            # Store final state before breaking
            time_history.append(time)
            alt_history.append(alt)
            vs_history.append(vs)
            hs_history.append(hs)
            ang_history.append(current_angle_deg)
            vs_target_history.append(get_vs_target(0))  # Target at alt=0
            hs_target_history.append(get_hs_target(0))  # Target at alt=0
            nn_history.append(NN)
            break  # Exit loop

        time += dt

        # Print simulation status
        if time % 10 < dt or alt < 100:
            print(f"time={time:.2f}, alt={alt:.2f}, vs={vs:.2f}, hs={hs:.2f}, ang={current_angle_deg:.2f}, fuel={fuel:.2f}, weight={weight:.2f}, thrust_N={actual_thrust_N:.2f}, NN={NN:.3f}")

    # --- Landing results ---
    print("\nLanding statistics:")
    print(f"Simulation time: {time:.2f} seconds")
    print(f"Final altitude: {alt:.2f} m")
    print(f"Final vertical speed: {vs:.2f} m/s (+ve is down)")
    print(f"Final horizontal speed: {hs:.2f} m/s")
    print(f"Final angle: {current_angle_deg:.2f} degrees")
    print(f"Remaining fuel: {fuel:.2f} kg")

    # Evaluate landing success
    MAX_SAFE_VS = 2.5
    MAX_SAFE_HS = 2.5
    MAX_SAFE_ANGLE = 5
    
    if abs(vs) <= MAX_SAFE_VS and abs(hs) <= MAX_SAFE_HS and abs(current_angle_deg) <= MAX_SAFE_ANGLE:
        print(f"\nLanding successful! (vs={vs:.2f}, hs={hs:.2f}, ang={current_angle_deg:.2f})")
    else:
        print("\nLanding failed.")
        # Print failure reasons
        if abs(vs) > MAX_SAFE_VS: print(f"Reason: Vertical speed too high: {vs:.2f} m/s (max {MAX_SAFE_VS} m/s)")
        if abs(hs) > MAX_SAFE_HS: print(f"Reason: Horizontal speed too high: {hs:.2f} m/s (max {MAX_SAFE_HS} m/s)")
        if abs(current_angle_deg) > MAX_SAFE_ANGLE: print(f"Reason: Final angle too high: {current_angle_deg:.2f} degrees (max {MAX_SAFE_ANGLE} deg)")

    # --- Plotting ---
    if time_history:  # Check if there's data to plot
        fig, axs = plt.subplots(4, 1, figsize=(7, 7), sharex=True)

        # Plot 1: Vertical Speed
        axs[0].plot(time_history, vs_history, label='Actual VS', linewidth=1.5)
        axs[0].plot(time_history, vs_target_history, label='Target VS', linestyle='--', color='red')
        axs[0].set_ylabel("Vertical Speed (m/s)\n(+ve is down)")
        axs[0].set_title("PID Control Performance")
        axs[0].legend()
        axs[0].grid(True)
        axs[0].axhline(0, color='black', linewidth=0.5)  # Zero line for reference

        # Plot 2: Horizontal Speed
        axs[1].plot(time_history, hs_history, label='Actual HS', linewidth=1.5)
        axs[1].plot(time_history, hs_target_history, label='Target HS', linestyle='--', color='red')
        axs[1].set_ylabel("Horizontal Speed (m/s)")
        axs[1].legend()
        axs[1].grid(True)
        axs[1].axhline(0, color='black', linewidth=0.5)  # Zero line for reference

        # Plot 3: Altitude and Angle
        ax3b = axs[2].twinx()  # Create a second y-axis for angle
        axs[2].plot(time_history, alt_history, label='Altitude', color='green')
        axs[2].set_ylabel("Altitude (m)", color='green')
        axs[2].tick_params(axis='y', labelcolor='green')
        axs[2].legend(loc='upper left')
        axs[2].grid(True, axis='y')  # Grid only for altitude axis

        ax3b.plot(time_history, ang_history, label='Angle', color='purple', linestyle='-.')
        ax3b.set_ylabel("Angle (deg)", color='purple')
        ax3b.tick_params(axis='y', labelcolor='purple')
        ax3b.legend(loc='upper right')

        # Plot 4: Throttle (NN)
        axs[3].plot(time_history, nn_history, label='Throttle (NN)', color='orange')
        axs[3].set_ylabel("Throttle (0-1)")
        axs[3].set_xlabel("Time (s)")
        axs[3].legend()
        axs[3].grid(True)
        axs[3].set_ylim(0, 1.1)  # Set y-axis limit for throttle 0-1

        plt.tight_layout(rect=[0, 0.03, 1, 0.98])  # Adjust layout to prevent title overlap
        plt.show()
    else:
        print("No data collected for plotting.")


if __name__ == "__main__":
    simulate()