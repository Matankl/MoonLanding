
import math
import matplotlib.pyplot as plt

# Represents the Moon's properties and physics
class Moon:
    RADIUS = 3475 * 1000  # Moon radius in meters
    ACC = 1.622  # Gravitational acceleration on the Moon in m/s^2
    EQ_SPEED = 1700  # Orbital speed at the Moon's equator in m/s

    @staticmethod
    def get_acc(speed):
        # Computes the gravitational effect based on horizontal speed
        n = abs(speed) / Moon.EQ_SPEED
        return (1 - n) * Moon.ACC


# Represents the Bereshit lander and its behavior during landing
class Bereshit101:
    def __init__(self):
        self.WEIGHT_EMP = 165
        self.WEIGHT_FUEL = 420
        self.WEIGHT_FULL = self.WEIGHT_EMP + self.WEIGHT_FUEL

        self.MAIN_ENG_F = 430
        self.SECOND_ENG_F = 25
        self.MAIN_BURN = 0.15
        self.SECOND_BURN = 0.009
        self.ALL_BURN = self.MAIN_BURN + 8 * self.SECOND_BURN

        # PID control constants (tuned)
        self.last_err = 0
        self.P = 0.05
        self.I = 0.05
        self.D = 0.1
        self.i = 0

        # For plotting
        self.time_log = []
        self.vs_log = []
        self.wanted_vs_log = []
        self.nn_log = []
        self.angle_log = []
        self.hs_log = []

    def acc_max(self, weight):
        return self.acc(weight, True, 8)

    def acc(self, weight, main, seconds):
        t = self.MAIN_ENG_F if main else 0
        t += seconds * self.SECOND_ENG_F
        return t / weight

    def compute_wanted_vs(self, alt, start_alt):
        # Normalize: 1 at start, 0 at ground
        x = alt / start_alt
        return 1 + 29 / (1 + math.exp((x - 0.5) * 10))

    def update(self, vs, dt, wanted_vs):
        p = vs - wanted_vs
        d = (p - self.last_err) / dt
        self.i += p * dt
        self.last_err = p
        NN = p * self.P + self.i * self.I + d * self.D
        return max(0.0, min(1.0, NN))

    def simulate_landing(self):
        print("\nSimulating Bereshit's Landing:\n")
        print("{:<6} {:<6} {:<6} {:<8} {:<8} {:<6} {:<8} {:<6} {:<6}".format(
            "Time", "VS", "HS", "Distance", "Altitude", "Angle", "Weight", "Acc", "Fuel"))
        print("-" * 80)

        vs = 24.8
        hs = 932
        dist = 181 * 1000
        ang = 58.3
        alt = 13748
        start_alt = alt
        time = 0
        dt = 1
        acc = 0
        fuel = 121
        weight = self.WEIGHT_EMP + fuel
        NN = 0.7
        wanted_vs = 30  # initial target

        while alt > 0:
            # Log data for plotting
            self.time_log.append(time)
            self.vs_log.append(vs)
            self.nn_log.append(NN)
            self.wanted_vs_log.append(wanted_vs)
            self.angle_log.append(ang)
            self.hs_log.append(hs)

            # Print every 10s or near ground
            if time % 10 == 0 or alt < 100:
                print("{:<6} {:<6.2f} {:<6.2f} {:<8.2f} {:<8.2f} {:<6.2f} {:<8.2f} {:<6.2f} {:<6.2f}".format(
                    time, vs, hs, dist, alt, ang, weight, acc, fuel))

            # Update wanted vertical speed based on altitude
            if alt > 4000:
                wanted_vs = 24
            elif alt > 2000:
                wanted_vs = 20
            elif alt > 500:
                wanted_vs = 16
            elif alt > 100:
                wanted_vs = 10
            elif alt > 20:
                wanted_vs = 6
            else:
                wanted_vs = 1

            # wanted_vs = self.compute_wanted_vs(alt, start_alt)

            if vs == wanted_vs:
                self.i = 0

            # Compute thrust power using PID for vertical speed
            NN = self.update(vs, dt, wanted_vs)

            # Fuel & acceleration
            dw = dt * self.ALL_BURN * NN
            if fuel > 0:
                fuel -= dw
                weight = self.WEIGHT_EMP + fuel
                acc = NN * self.acc_max(weight)
            else:
                acc = 0

            # Compute predictive angle to slow down horizontal speed
            if alt > 5 and vs > 0.1 and acc > 0:
                time_to_land = alt / vs
                required_h_acc = hs / max(time_to_land, 0.1)
                desired_ang_rad = math.asin(min(1, max(0, required_h_acc / acc)))
                ang = math.degrees(desired_ang_rad)
                ang = max(5, min(60, ang))  # Clamp
            else:
                ang = 0  # Vertical landing

            # Apply physics
            ang_rad = math.radians(ang)
            h_acc = math.sin(ang_rad) * acc
            v_acc = math.cos(ang_rad) * acc
            vacc = Moon.get_acc(hs)

            time += dt

            v_acc -= vacc
            if hs > 0:
                hs -= h_acc * dt
            dist -= hs * dt
            vs -= v_acc * dt
            alt -= dt * vs

        self.plot_data()

    def plot_data(self):
        # Plotting the results
        plt.figure(figsize=(12, 10))

        # Vertical speed
        plt.subplot(4, 1, 1)
        plt.plot(self.time_log, self.vs_log, label="Actual Vertical Speed (vs)")
        plt.plot(self.time_log, self.wanted_vs_log, label="Wanted Vertical Speed", linestyle='--')
        plt.ylabel("Vertical Speed (m/s)")
        plt.title("Vertical Speed vs Time")
        plt.legend()
        plt.grid(True)

        # Horizontal speed
        plt.subplot(4, 1, 2)
        plt.plot(self.time_log, self.hs_log, label="Horizontal Speed (hs)", color="red")
        plt.ylabel("Horizontal Speed (m/s)")
        plt.title("Horizontal Speed vs Time")
        plt.legend()
        plt.grid(True)

        # Thrust power
        plt.subplot(4, 1, 3)
        plt.plot(self.time_log, self.nn_log, label="Thrust Power (NN)", color="orange")
        plt.xlabel("Time (s)")
        plt.ylabel("NN")
        plt.title("Thrust Power vs Time")
        plt.ylim(0, 1.1)
        plt.legend()
        plt.grid(True)

        # Engine angle
        plt.subplot(4, 1, 4)
        plt.plot(self.time_log, self.angle_log, label="Engine Angle (degrees)", color="green")
        plt.xlabel("Time (s)")
        plt.ylabel("Angle (°)")
        plt.title("Engine Angle vs Time")
        plt.ylim(0, 60)
        plt.legend()
        plt.grid(True)

        plt.tight_layout()
        plt.show()


# Entry point
if __name__ == "__main__":
    lander = Bereshit101()
    lander.simulate_landing()


### TOD0:
    # zero i when we reach the wanted state
    # Generate a continuous function instead of modes



