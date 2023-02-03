/**
 * PD control.
 * 
 * Input:  Roll (rad)
 * Output: Steering angle rate (rad/s)
 * 
 * @param rollReference Desired roll (rad)
 * @param roll Current roll (rad)
 * @param rollRate Current roll rate (rad/s)
 * @param Kp PD proportionality constant (s⁻¹)
 * @param Kd PD derivative constant (1)
 * 
 * @author Ossian Eriksson
*/
extern double balancingController(double rollReference, double roll, double rollRate, double Kp, double Kd) {
    return -(Kp * (roll - rollReference) + Kd * rollRate);
}
