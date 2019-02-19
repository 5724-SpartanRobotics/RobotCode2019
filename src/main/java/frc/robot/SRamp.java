package frc.robot;
 
/// <summary>
/// S-Curve
/// ramping rate is changed linearly based on current error, (e.g. Setpoint - Output)
/// </summary>
public class SRamp
{
	public double Setpoint;
    private double _Output;

	/// <summary>
	/// Gets or sets the rate.
	/// The rate is in units added or subtracted with each call to update. This is the max rate. The actual
	/// rate is rampped based upon the error.
	/// </summary>
	/// <value>
	/// The rate.
	/// </value>
	public double Rate;
    
    private Ramp _VelRamp;
    
    /// <summary>
	/// Gets the current rate.
	/// This is the calculated rate based upon the current error
	/// </summary>
	/// <value>
	/// The current rate.
	/// </value>
	public double getCurrentRate()
	{
		return _VelRamp.getOutput();
    }
    
    public double getOutput() {
        return _Output;
    }
    
	/// <summary>
	/// Gets or sets the maximum acceleration rate.
	/// This is the ammount the acceleration can increase or decrease by with each update.
	/// Defaults to 2.
	/// </summary>
	/// <value>
	/// The maximum access rate.
	/// </value>
	public double getMaxAccelRate()
	{
		return _VelRamp.Rate;
    }
    
	public void setMaxAccelRate(double value)
	{
        _VelRamp.Rate = value;
    }

	public SRamp()
	{
		_VelRamp = new Ramp();
		_VelRamp.Rate = 5;
	}

	public void update()
	{
		double accelSetpoint = calculateVelRampSetpoint();
		//set the new rate setpoint
		_VelRamp.Setpoint = accelSetpoint;
		_VelRamp.Update();

		if (Setpoint != _Output)
		{
			_Output += getCurrentRate();
		}
		else
		{
			_Output = Setpoint;
		}
	}

	// Sets the output (and setpoint) to the desired value immediately (needed when transitioning from auto to teleop through disabled)
	public void setOutput(double value)
	{
		Setpoint = value;
		_Output = value;
	}

	private double calculateVelRampSetpoint()
	{
		//based upon the difference between our current output and our desired setpoint
		// determine if our Accel ramp should be increasing toward our Rate, decreasing
		// toward the negative of our Rate, or moving toward 0.
		double error = Setpoint - _Output;
		//at the current s-curve rate, how many updates would it take to get to a rate of zero
		// (note the current rate could be negative, positive or 0)
		double numToGetToZeroVel = Math.abs(getCurrentRate() / getMaxAccelRate());
		//total output change if we were to go to a zero accel rate now
		double totalChangeIfAtSetpoint = getCurrentRate() * numToGetToZeroVel / 2;
		if (error > 0)
		{
			if (error > (totalChangeIfAtSetpoint + getMaxAccelRate()))
			{
				return Rate;
			}
			else
			{
				return 0;
			}
		}
		else if (error < 0)
		{
			if (error < (totalChangeIfAtSetpoint - getMaxAccelRate()))
			{
				return Rate * -1;
			}
			else
			{
				return 0;
			}
		}
		else
		{
			return 0;
		}
	}

}