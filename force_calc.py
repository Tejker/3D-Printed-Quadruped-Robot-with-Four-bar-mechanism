import numpy as np
import matplotlib.pyplot as plt

# --- 1. Hardcoded Parameters (Modify these as needed) ---
W = 100.0          # Assumed Weight (N)
F_cr = 78.453      # Crank Input Force (N)
theta_deg = 0.0    # Assumed theta (degrees)

# Link lengths (mm)
L_AB = 93.0
L_BC = 30.0
L_CD = 96.0
L_AD = 25.0

# --- 2. Input: Alpha restricted range ---
# Sweeping alpha from -41 to 49 degrees
alpha_deg = np.linspace(-41, 49, 200)
alpha = np.radians(alpha_deg)
theta = np.radians(theta_deg)

# --- 3. Solve Quadrilateral (Alpha to Delta) ---
# Apply your specific geometric rule: angle CBA = alpha + 90
angle_CBA_deg = alpha_deg + 90
angle_CBA_rad = np.radians(angle_CBA_deg)

# Vector BA points along the negative x-axis (180 degrees or pi radians).
# The absolute angle of link BC relative to the positive x-axis is (180 - CBA).
angle_BC_rad = np.pi - angle_CBA_rad

# Calculate coordinates of Joint C
Cx = L_AB + L_BC * np.cos(angle_BC_rad)
Cy = L_BC * np.sin(angle_BC_rad)

# Inverse Kinematics math to find the required crank angle Delta
K = (Cx**2 + Cy**2 + L_AD**2 - L_CD**2) / (2 * L_AD)
R = np.sqrt(Cx**2 + Cy**2)
phi = np.arctan2(Cy, Cx)

# Calculate delta (Using the '+' assembly mode)
delta = phi + np.arccos(K / R)

# --- 4. Your Force Equations ---
# Calculate parallel and perpendicular components of F_cr
Fc_para = F_cr * np.sin(delta)
Fc_perp = F_cr * np.cos(delta)

# Calculate the angle term: 90 - (alpha - theta)
angle_term = (np.pi / 2) - (alpha - theta)

# Calculate Reaction Forces Rx and Ry
Rx = (W / 2) * np.cos(angle_term) - Fc_para * np.sin(alpha) + Fc_perp * np.cos(alpha)
Ry = (W / 2) * np.sin(angle_term) + Fc_para * np.cos(alpha) + Fc_perp * np.sin(alpha)

# ---> ADD THESE TWO LINES HERE <---
Fx_prime = Rx * np.sin(alpha) + Ry * np.cos(alpha)
Fy_prime = -Ry * np.sin(alpha) + Rx * np.cos(alpha)

# --- 5. Plotting ---
plt.figure(figsize=(10, 6))

plt.plot(alpha_deg, Rx, 'b-', label='Rx', linewidth=2)
plt.plot(alpha_deg, Ry, 'r-', label='Ry', linewidth=2)
plt.plot(alpha_deg, Fc_para, 'g--', label='Fc Parallel (F_c||)')
plt.plot(alpha_deg, Fc_perp, 'm--', label='Fc Perpendicular (F_c_perp)')
plt.plot(alpha_deg, Fx_prime, 'c-.', label="Fx' (Thigh Local)", linewidth=2)
plt.plot(alpha_deg, Fy_prime, 'y-.', label="Fy' (Thigh Local)", linewidth=2)

plt.title('Leg Force Analysis: $\\angle CBA = \\alpha + 90^\\circ$')
plt.xlabel('Leg Output Angle $\\alpha$ (degrees)')
plt.ylabel('Force (N)')
plt.grid(True)
plt.legend()
plt.xlim(-41, 49)
plt.show()

# --- 6. Theta Sweep for Maximum Force ---
# Sweeping theta from -180 to 180 degrees (no restriction)
theta_sweep_deg = np.linspace(-180, 180, 720)

global_max_force = 0
worst_case_theta = 0
critical_force_type = ""
critical_alpha = 0

# Fc_para and Fc_perp don't change with theta, so we find their maxes once
max_fc_para = np.max(np.abs(Fc_para))
max_fc_perp = np.max(np.abs(Fc_perp))

for t_deg in theta_sweep_deg:
    t_rad = np.radians(t_deg)
    
    # Recalculate angle term with the new swept theta
    angle_term_sweep = (np.pi / 2) - (alpha - t_rad)
    
    # Recalculate Rx and Ry arrays for this specific theta
    Rx_sweep = (W / 2) * np.cos(angle_term_sweep) - Fc_para * np.sin(alpha) + Fc_perp * np.cos(alpha)
    Ry_sweep = (W / 2) * np.sin(angle_term_sweep) + Fc_para * np.cos(alpha) + Fc_perp * np.sin(alpha)
    Fx_prime_sweep = Rx_sweep * np.sin(alpha) + Ry_sweep * np.cos(alpha)
    Fy_prime_sweep = -Ry_sweep * np.sin(alpha) + Rx_sweep * np.cos(alpha)
    
    # Find the maximum absolute force among all 4 in this sweep iteration
    max_rx = np.max(np.abs(Rx_sweep))
    max_ry = np.max(np.abs(Ry_sweep))
    max_fx_prime = np.max(np.abs(Fx_prime_sweep))
    max_fy_prime = np.max(np.abs(Fy_prime_sweep))
    
    current_max = max(max_rx, max_ry, max_fc_para, max_fc_perp, max_fx_prime, max_fy_prime)
    
    
    # Update global tracker if we found a new absolute maximum
    if current_max > global_max_force:
        global_max_force = current_max
        worst_case_theta = t_deg
        
        # Identify exactly which force spiked and at what alpha position
        if current_max == max_rx:
            critical_force_type = "Rx"
            critical_alpha = alpha_deg[np.argmax(np.abs(Rx_sweep))]
        elif current_max == max_ry:
            critical_force_type = "Ry"
            critical_alpha = alpha_deg[np.argmax(np.abs(Ry_sweep))]
        elif current_max == max_fc_para:
            critical_force_type = "Fc Parallel"
            critical_alpha = alpha_deg[np.argmax(np.abs(Fc_para))]
        else:
            critical_force_type = "Fc Perpendicular"
            critical_alpha = alpha_deg[np.argmax(np.abs(Fc_perp))]

print(f"\n--- THETA SWEEP RESULTS ---")
print(f"Absolute Maximum Force : {global_max_force:.3f} N")
print(f"Force Type             : {critical_force_type}")
print(f"Occurs at Theta        : {worst_case_theta:.2f}°")
print(f"Occurs at Leg Alpha    : {critical_alpha:.2f}°")

# --- 7. Interactive Theta Slider Plot ---
from matplotlib.widgets import Slider

interactive_theta_deg = theta_deg
interactive_theta = np.radians(interactive_theta_deg)
interactive_angle_term = (np.pi / 2) - (alpha - interactive_theta)

interactive_Rx = (W / 2) * np.cos(interactive_angle_term) - Fc_para * np.sin(alpha) + Fc_perp * np.cos(alpha)
interactive_Ry = (W / 2) * np.sin(interactive_angle_term) + Fc_para * np.cos(alpha) + Fc_perp * np.sin(alpha)
interactive_Fx_prime = interactive_Rx * np.sin(alpha) + interactive_Ry * np.cos(alpha)
interactive_Fy_prime = -interactive_Ry * np.sin(alpha) + interactive_Rx * np.cos(alpha)

fig, ax = plt.subplots(figsize=(10, 6))
plt.subplots_adjust(bottom=0.22)

line_Rx, = ax.plot(alpha_deg, interactive_Rx, 'b-', label='Rx', linewidth=2)
line_Ry, = ax.plot(alpha_deg, interactive_Ry, 'r-', label='Ry', linewidth=2)
line_Fc_para, = ax.plot(alpha_deg, Fc_para, 'g--', label='Fc Parallel (F_c||)')
line_Fc_perp, = ax.plot(alpha_deg, Fc_perp, 'm--', label='Fc Perpendicular (F_c_perp)')
line_Fx_prime, = ax.plot(alpha_deg, interactive_Fx_prime, 'c-.', label="Fx' (Thigh Local)", linewidth=2)
line_Fy_prime, = ax.plot(alpha_deg, interactive_Fy_prime, 'y-.', label="Fy' (Thigh Local)", linewidth=2)

ax.set_title(f'Leg Force Analysis: theta = {interactive_theta_deg:.1f} degrees')
ax.set_xlabel('Leg Output Angle $\\alpha$ (degrees)')
ax.set_ylabel('Force (N)')
ax.grid(True)
ax.legend()
ax.set_xlim(-41, 49)

theta_slider_ax = plt.axes([0.15, 0.08, 0.7, 0.04])
theta_slider = Slider(
    ax=theta_slider_ax,
    label='Theta (degrees)',
    valmin=-180,
    valmax=180,
    valinit=interactive_theta_deg,
    valstep=1
)

def update_theta(val):
    slider_theta_deg = theta_slider.val
    slider_theta = np.radians(slider_theta_deg)
    slider_angle_term = (np.pi / 2) - (alpha - slider_theta)

    slider_Rx = (W / 2) * np.cos(slider_angle_term) - Fc_para * np.sin(alpha) + Fc_perp * np.cos(alpha)
    slider_Ry = (W / 2) * np.sin(slider_angle_term) + Fc_para * np.cos(alpha) + Fc_perp * np.sin(alpha)
    slider_Fx_prime = slider_Rx * np.sin(alpha) + slider_Ry * np.cos(alpha)
    slider_Fy_prime = -slider_Ry * np.sin(alpha) + slider_Rx * np.cos(alpha)

    line_Rx.set_ydata(slider_Rx)
    line_Ry.set_ydata(slider_Ry)
    line_Fx_prime.set_ydata(slider_Fx_prime)
    line_Fy_prime.set_ydata(slider_Fy_prime)
    ax.set_title(f'Leg Force Analysis: theta = {slider_theta_deg:.1f} degrees')
    ax.relim()
    ax.autoscale_view(scalex=False, scaley=True)
    fig.canvas.draw_idle()

theta_slider.on_changed(update_theta)
plt.show()
