from cc.udp import UDP
from loop_rate_limiters import RateLimiter

from src.iteh import Humanoid
from src.iteh.policy import RlController, PolicyConfig

def main():
    config = PolicyConfig.from_arguments()
    print(f"Policy frequency: {1 / config.policy_dt} Hz")
    udp = UDP(("0.0.0.0", 11000), ("172.28.0.5", 11000))

    # Initialize and start policy controller
    controller = RlController(config)
    controller.load_policy()
    # Initialize rate limiter of the policy loop to match the policy frequency
    rate = RateLimiter(1 / config.policy_dt)
    # Initialize the robot and enter damping mode
    robot = Humanoid()
    robot.enter_damping()
    obs = robot.reset()

    try:
        while True:
            actions = controller.update(obs)
            obs = robot.step(actions)
            udp.send_numpy(obs)

            rate.sleep()

    except KeyboardInterrupt:
        print("\n[INFO] locomotion stopped by user.")
    finally:
        robot.stop()
        print("[INFO] robot stopped.")

if __name__ == "__main__":
    main()
