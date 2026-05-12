import sys
import time
from stretch4_body.robot.robot_client import RobotClient

def main():
    try:
        r = RobotClient()
        if not r.startup():
            print("Failed to start RobotClient")
            sys.exit(1)
            
        print("\n--- Sending test routine to server ---")
        # finished, rid = r.routines.routine_blind_dock(wait_on_completion=False)
        finished, rid = r.routines.routine_robot_home(wait_on_completion=False)
        print(f"Test routine successfully pushed with ID: {rid}")
        
        print("Waiting 4 seconds to observe routine doing work...")
        for i in range(4):
            time.sleep(1)
            print(f"  Wait {i+1} / 4...")
            
        print("\n--- Triggering cancel using ID ---")
        r.routines.cancel_routine(id=str(rid))
        
        print("Waiting 3 seconds to see if it gracefully exits on the server side...")
        time.sleep(3)
        
        print("Done. Client shutting down.")
        r.stop()
    except KeyboardInterrupt:
        if 'r' in locals():
            r.stop()

if __name__ == '__main__':
    main()
