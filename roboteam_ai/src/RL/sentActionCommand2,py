import os
import sys
import subprocess

def send_action_command(num_attacker, num_defender, num_waller):
    # Path to the C++ executable
    current_dir = os.path.dirname(os.path.abspath(__file__))
    cpp_executable_path = os.path.join(current_dir, "receiveActionCommand")  # Adjust if necessary

    # Start the subprocess
    process = subprocess.Popen(
        [cpp_executable_path],
        stdin=subprocess.PIPE,  # Open a pipe to the C++ program
        text=True  # Use text mode for easier communication
    )

    # Format the data as a comma-separated string
    message = f"{num_defender},{num_attacker},{num_waller}\n"

    # Write the message to the subprocess stdin
    process.stdin.write(message)
    process.stdin.flush()

    print(f"Sent: {message.strip()}")

    # Close the stdin to signal EOF to the subprocess
    process.stdin.close()
    process.wait()

if __name__ == "__main__":
    send_action_command(2, 3, 1)
