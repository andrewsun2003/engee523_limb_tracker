import serial
import csv
import statistics
import time

# Configuration
SERIAL_PORT = "COM14"  # Change as needed
BAUD_RATE = 115200
NUM_SAMPLES = 4
OUTPUT_FILE = "C:/Users/as716/Downloads/engee523_limb_tracker-main/sensor_fusion/calibrated_imu_roll.csv"

# Open serial connection
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
time.sleep(2)  # Wait for connection

# Open CSV file
with open(OUTPUT_FILE, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow([
        "Reference_Angle",
        "Mean_EKF_Roll", "Std_EKF_Roll", "EKF_Roll_Bias",
        "Mean_DMP_Roll", "Std_DMP_Roll", "DMP_Roll_Bias"
    ])

    print("Ready to log roll data at reference angles.")
    print("Press Ctrl+C to exit.\n")

    try:
        while True:
            ref_angle = float(input("Enter reference angle (°): ").strip())

            ekf_rolls, dmp_rolls = [], []

            print(f"Press ENTER to collect each of the {NUM_SAMPLES} roll samples...")

            while len(ekf_rolls) < NUM_SAMPLES:
                input(f"Sample {len(ekf_rolls) + 1} - press ENTER when ready")
                ser.reset_input_buffer()
                time.sleep(0.05)
                valid_sample = False

                # Keep reading until a valid line is received
                while not valid_sample:
                    if ser.in_waiting:
                        line = ser.readline().decode(errors='ignore').strip()
                        parts = line.split('\t')

                        if len(parts) == 19:
                            try:
                                eroll = float(parts[13])
                                droll = float(parts[16])

                                ekf_rolls.append(eroll)
                                dmp_rolls.append(droll)
                                valid_sample = True

                                print(f"Recorded: EKF_Roll={eroll:.2f}, DMP_Roll={droll:.2f}")

                            except ValueError:
                                continue

            # Compute stats and bias
            def stats_bias(values):
                mean = statistics.mean(values)
                std = statistics.stdev(values)
                bias = mean - ref_angle
                return round(mean, 2), round(std, 2), round(bias, 2)

            row = [
                ref_angle,
                *stats_bias(ekf_rolls),
                *stats_bias(dmp_rolls)
            ]
            writer.writerow(row)
            print(f"Saved roll data for {ref_angle}°\n")

    except KeyboardInterrupt:
        print("\nData logging stopped.")
