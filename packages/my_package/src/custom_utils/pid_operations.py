from duckietown_msgs.msg import WheelsCmdStamped

class PIDOperations:
    _instance = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(PIDOperations, cls).__new__(cls)
        return cls._instance
    
    @staticmethod
    def getForwardPIDWheelMsg(base_velocity, error_last, integration_stored, error, proportional_gain, derivative_gain, integral_gain, integral_saturation):
        P = error*proportional_gain
        errorRateOfChange = error - error_last
        D = derivative_gain * errorRateOfChange
        integration_stored_update = integration_stored + error
        integration_stored_updated = (integration_stored_update) if abs(integration_stored_update) <= integral_saturation else (integration_stored_update/integration_stored_update)*integral_saturation
        I = integral_gain * integration_stored

        error_last_updated = error
        update = P + I + D

        if update < 0:
            message = WheelsCmdStamped(vel_left=base_velocity, vel_right=base_velocity+abs(update))
        elif update > 0:
            message = WheelsCmdStamped(vel_left=base_velocity+abs(update), vel_right=base_velocity)
        else:
            message = WheelsCmdStamped(vel_left=base_velocity, vel_right=base_velocity)

        return integration_stored_updated, error_last_updated, message

    @staticmethod
    def getRotatePIDWheelMsg(error_last, integration_stored, error, proportional_gain, derivative_gain, integral_gain, integral_saturation):
        P = error*proportional_gain
        errorRateOfChange = error - error_last
        D = derivative_gain * errorRateOfChange
        integration_stored_update = integration_stored + error
        integration_stored_updated = (integration_stored_update) if abs(integration_stored_update) <= integral_saturation else (integration_stored_update/integration_stored_update)*integral_saturation
        I = integral_gain * integration_stored

        error_last_updated = error
        update = P + I + D

        if update < 0:
            message = WheelsCmdStamped(vel_left=-abs(update), vel_right=abs(update))
        elif update > 0:
            message = WheelsCmdStamped(vel_left=abs(update), vel_right=-abs(update))
        else:
            message = WheelsCmdStamped(vel_left=0, vel_right=0)

        return integration_stored_updated, error_last_updated, message
