from simple_pid import PID

def pid_control(Kp, Ki, Kd, setpoint, current_value, output_limits):
    """
    This function initializes a PID controller with specified gains and setpoints, computes the control
    output based on the current system value, and returns the calculated control signal.

    Parameters:
        Kp (float): Proportional gain.
        Ki (float): Integral gain.
        Kd (float): Derivative gain.
        setpoint (float): Target value the system tries to achieve.
        current_value (float): Current state of the system.
        output_limits (tuple): Min and max values for the control output.

    Returns:
        float: The control output to adjust the system.
    """
    # Initialize the PID controller with given gains
    pid = PID(Kp=Kp, Ki=Ki, Kd=Kd, setpoint=setpoint)

    # Set the output limits for the PID
    pid.output_limits = output_limits

    # Calculate and return the control output
    control = pid(current_value)
    return control