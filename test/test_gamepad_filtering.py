#!/usr/bin/env python3
import time
import unittest
from stretch4_body.core.gamepad_controller import GamePadController, GPEvent

class MockGamePadController(GamePadController):
    def __init__(self):
        # Initialize without calling startup or starting threads/dev
        super().__init__(print_events=False)
        self.mock_events = []
        # Bypass hardware checks
        self.is_gamepad_active = True
        self.dev = True # Just to pass truthy check if needed, though we override get_gamepad_events

    def get_gamepad_events(self):
        events = self.mock_events
        self.mock_events = []
        return events

    def inject_events(self, events):
        self.mock_events.extend(events)

class TestGamepadFiltering(unittest.TestCase):
    def test_filtering_logic(self):
        controller = MockGamePadController()
        
        # 1. Initial State: Should be None (inactive)
        # Note: In __init__, it calls get_state() once. At that time last_event_ts is 0.
        # But zero_state_sent_counter is 0.
        # Let's see. In init: self.last_event_ts = 0.
        # get_state: time.time() - 0 > 0.5 (Active=False).
        # State is zero. Active=False.
        # zero_state_sent_counter (0) < 5 -> increments to 1, returns state.
        # So actually, initially it might return 5 frames of zeros if queried immediately.
        # Let's just consume the initial 5 frames.
        print("Consuming initial frames...")
        for _ in range(6):
            controller.update()
        
        state = controller.get_state()
        self.assertIsNone(state, "Should be None after initialization and flush")

        # 2. Inject Button Press
        print("Injecting Button Press...")
        # Simulating BTN_SOUTH (bottom button) press
        # Note: The controller logic uses "if 'BTN_SOUTH' in list(event.code):", implying code might be a list (aliasing).
        # We simulate this by passing a list.
        controller.inject_events([GPEvent(code=['BTN_SOUTH'], state=1, ev_type='EV_KEY')])
        controller.update()
        state = controller.get_state()
        self.assertIsNotNone(state, "Should return state after button press")
        self.assertTrue(state['bottom_button_pressed'], "Bottom button should be pressed")

        # 3. Hold Button (No new events, but button is pressed)
        # Wait > 0.5s to ensure time check fails
        print("Waiting > 0.5s to test hold logic...")
        time.sleep(0.6)
        controller.update() # No events
        state = controller.get_state()
        self.assertIsNotNone(state, "Should return state while holding button")
        self.assertTrue(state['bottom_button_pressed'], "Bottom button should still be considered pressed")

        # 4. Release Button
        print("Injecting Button Release...")
        controller.inject_events([GPEvent(code=['BTN_SOUTH'], state=0, ev_type='EV_KEY')])
        controller.update()
        
        # Wait for activity timeout to expire so we enter "stop frame" counting mode
        print("Waiting for activity timeout (0.6s)...")
        time.sleep(0.6)
        
        # 5. Verify 5 stop frames
        print("Verifying stop frames...")
        for i in range(5):
            state = controller.get_state()
            self.assertIsNotNone(state, f"Stop frame {i+1} should be valid state")
            self.assertFalse(state['bottom_button_pressed'], "Button should be released in stop frame")
            # We need to call update to potentially increment logic if it depended on update loop?
            # Actually get_state handles the counter increment logic, update handles event processing.
            # In real usage, update is called in loop, get_state is called by user.
            # But the logic updates counter IN get_state.
            
        # 6. Verify None after 5 frames
        state = controller.get_state()
        self.assertIsNone(state, "Should return None after stop frames")

        # 7. Joystick Move
        print("Injecting Joystick Move...")
        # ABS_X
        # GamePadController normalizes, so let's send a large int
        controller.inject_events([GPEvent(code='ABS_X', state=30000, ev_type='EV_ABS')])
        controller.update()
        state = controller.get_state()
        self.assertIsNotNone(state)
        self.assertNotEqual(state['left_stick_x'], 0)

        # 8. Hold Joystick (No events)
        time.sleep(0.6)
        controller.update()
        state = controller.get_state()
        self.assertIsNotNone(state, "Should return state while holding joystick")
        self.assertNotEqual(state['left_stick_x'], 0)

        # 9. Release Joystick
        print("Injecting Joystick Release...")
        controller.inject_events([GPEvent(code='ABS_X', state=0, ev_type='EV_ABS')])
        controller.update()
        
        # Wait for activity timeout
        print("Waiting for activity timeout (0.6s)...")
        time.sleep(0.6)

        # Verify 5 stop frames
        for i in range(5):
            state = controller.get_state()
            self.assertIsNotNone(state)
            self.assertEqual(state['left_stick_x'], 0)
            
        state = controller.get_state()
        self.assertIsNone(state, "Should return None after stop frames for joystick")

if __name__ == '__main__':
    unittest.main()
