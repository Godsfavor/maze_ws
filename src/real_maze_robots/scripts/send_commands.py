#!/usr/bin/env python3
"""
Multi-Domain Command Sender
Sends /left or /right commands to robots across multiple ROS_DOMAIN_IDs
"""

import subprocess
import sys
import os

class MultiDomainCommander:
    def __init__(self, domain_ids):
        self.domain_ids = domain_ids
        print("=" * 50)
        print("🎮 Multi-Domain Robot Commander")
        print("=" * 50)
        print(f"Target ROS_DOMAIN_IDs: {domain_ids}")
        print("\nCommands:")
        print("  'l' or 'left'  : Send LEFT signal")
        print("  'r' or 'right' : Send RIGHT signal")
        print("  'q' or 'quit'  : Exit")
        print("=" * 50)
    
    def send_command(self, topic):
        """Send command to all configured ROS domains"""
        print(f"\n📡 Broadcasting /{topic} to domains {self.domain_ids}...")
        
        for domain_id in self.domain_ids:
            cmd = [
                'bash', '-c',
                f'source /opt/ros/humble/setup.bash && '
                f'export ROS_DOMAIN_ID={domain_id} && '
                f'ros2 topic pub /{topic} example_interfaces/msg/Empty "{{}}" --once'
            ]
            
            try:
                result = subprocess.run(
                    cmd,
                    capture_output=True,
                    text=True,
                    timeout=3
                )
                
                if result.returncode == 0:
                    print(f"  ✓ Domain {domain_id}: SUCCESS")
                else:
                    print(f"  ✗ Domain {domain_id}: FAILED")
                    if result.stderr:
                        print(f"    Error: {result.stderr.strip()}")
            
            except subprocess.TimeoutExpired:
                print(f"  ✗ Domain {domain_id}: TIMEOUT")
            except Exception as e:
                print(f"  ✗ Domain {domain_id}: {e}")
        
        print("✅ Command broadcast complete\n")
    
    def run(self):
        """Main command loop"""
        while True:
            try:
                user_input = input("Enter command (l/r/q): ").strip().lower()
                
                if user_input in ['l', 'left']:
                    self.send_command('left')
                elif user_input in ['r', 'right']:
                    self.send_command('right')
                elif user_input in ['q', 'quit', 'exit']:
                    print("👋 Goodbye!")
                    break
                else:
                    print("❌ Invalid command. Use 'l', 'r', or 'q'")
            
            except KeyboardInterrupt:
                print("\n👋 Interrupted. Goodbye!")
                break
            except EOFError:
                print("\n👋 EOF. Goodbye!")
                break


if __name__ == '__main__':
    # Default domain IDs (modify as needed)
    default_domains = [1, 3]  # Robot 1 and Robot 2
    
    # Parse command line arguments
    if len(sys.argv) > 1:
        try:
            domain_ids = [int(d) for d in sys.argv[1:]]
        except ValueError:
            print("Usage: python3 send_commands.py [domain_id1] [domain_id2] ...")
            print(f"Example: python3 send_commands.py 1 3 4")
            sys.exit(1)
    else:
        domain_ids = default_domains
    
    commander = MultiDomainCommander(domain_ids)
    commander.run()
