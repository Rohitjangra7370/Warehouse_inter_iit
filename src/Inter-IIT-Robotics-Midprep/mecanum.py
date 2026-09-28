import pandas as pd

def generate_mecanum_table(length_mm, width_mm, wheel_radius_mm):
    """
    Generates a table of wheel velocities for a 4-wheel Mecanum robot
    based on the kinematic model.
    """
    # 1. Convert dimensions to meters (SI units)
    L = length_mm / 1000.0  # Length
    W = width_mm / 1000.0   # Width
    R = wheel_radius_mm / 1000.0 # Wheel Radius

    # 2. Calculate Geometry Factor (k)
    # lx and ly are half-distances from the center
    lx = W / 2.0
    ly = L / 2.0
    k = lx + ly  # Sum of half-width and half-length

    # 3. Define Test Velocities
    v_test = 1.0  # m/s for linear movements
    w_test = 1.0  # rad/s for rotation

    # 4. Define All Motion Cases
    # Format: [Name, vx, vy, wz]
    # vx: Forward(+)/Backward(-)
    # vy: Left(+)/Right(-)
    # wz: Rotate CCW(+)/CW(-)
    cases = [
        {"Case": "Forward",             "vx": v_test,  "vy": 0,       "wz": 0},
        {"Case": "Backward",            "vx": -v_test, "vy": 0,       "wz": 0},
        {"Case": "Left (Slide)",        "vx": 0,       "vy": v_test,  "wz": 0},
        {"Case": "Right (Slide)",       "vx": 0,       "vy": -v_test, "wz": 0},
        {"Case": "Diag. Forward-Left",  "vx": v_test,  "vy": v_test,  "wz": 0},
        {"Case": "Diag. Forward-Right", "vx": v_test,  "vy": -v_test, "wz": 0},
        {"Case": "Diag. Backward-Left", "vx": -v_test, "vy": v_test,  "wz": 0},
        {"Case": "Diag. Backward-Right","vx": -v_test, "vy": -v_test, "wz": 0},
        {"Case": "Rotate CW (Right)",   "vx": 0,       "vy": 0,       "wz": -w_test},
        {"Case": "Rotate CCW (Left)",   "vx": 0,       "vy": 0,       "wz": w_test},
    ]

    results = []

    # 5. Compute Inverse Kinematics for each case
    for case in cases:
        vx = case["vx"]
        vy = case["vy"]
        wz = case["wz"]

        # Inverse Kinematics Equations (Derived from Eq. 18 in paper)
        # Note: W1=FL, W2=FR, W3=BL, W4=BR (based on standard configuration)
        w1 = (1 / R) * (vx - vy - k * wz)
        w2 = (1 / R) * (vx + vy + k * wz)
        w3 = (1 / R) * (vx + vy - k * wz)
        w4 = (1 / R) * (vx - vy + k * wz)

        results.append({
            "Motion Case": case["Case"],
            "Vx (m/s)": vx,
            "Vy (m/s)": vy,
            "Wz (rad/s)": wz,
            "Wheel 1 (rad/s)": round(w1, 2),
            "Wheel 2 (rad/s)": round(w2, 2),
            "Wheel 3 (rad/s)": round(w3, 2),
            "Wheel 4 (rad/s)": round(w4, 2)
        })

    # 6. Create DataFrame and Display
    df = pd.DataFrame(results)
    return df

# --- Configuration ---
# User Parameters
length = 550  # mm
width = 450   # mm
radius = 100   # mm (Assumed standard radius, adjust if yours differs)

# Run Generation
df_results = generate_mecanum_table(length, width, radius)

# Display the Table
print(f"Kinematic Table for Robot Size: {length}x{width} mm")
print(df_results.to_markdown(index=False))