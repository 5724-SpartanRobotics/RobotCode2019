package frc.robot;

public class Ramp
{
    public double Setpoint;
    private double _Output;

    public double Rate;

	public Ramp()
	{
		Rate = 50;
	}

	public double getOutput() {
        return _Output;
    }
    
	public void Update()
	{
		if (_Output < (Setpoint - Rate))
		{
			_Output += Rate;
		}
		else if (_Output > (Setpoint + Rate))
		{
			_Output -= Rate;
		}
		else
		{
			_Output = Setpoint;
		}
	}
}
