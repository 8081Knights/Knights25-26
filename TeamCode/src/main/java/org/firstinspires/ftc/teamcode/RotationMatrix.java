package org.firstinspires.ftc.teamcode;

public class RotationMatrix {

    // Row-major 3x3 matrix
    public double[][] m = new double[3][3];

    public static RotationMatrix fromYawPitchRoll(double yaw, double pitch, double roll) {
        // FTC: yaw(Z), pitch(X), roll(Y)
        double cy = Math.cos(yaw);
        double sy = Math.sin(yaw);
        double cp = Math.cos(pitch);
        double sp = Math.sin(pitch);
        double cr = Math.cos(roll);
        double sr = Math.sin(roll);

        RotationMatrix R = new RotationMatrix();

        // R = Rz(yaw) * Rx(pitch) * Ry(roll)
        R.m[0][0] =  cy * cr + sy * sp * sr;
        R.m[0][1] = -cy * sr + sy * sp * cr;
        R.m[0][2] =  sy * cp;

        R.m[1][0] =  cp * sr;
        R.m[1][1] =  cp * cr;
        R.m[1][2] = -sp;

        R.m[2][0] = -sy * cr + cy * sp * sr;
        R.m[2][1] =  sy * sr + cy * sp * cr;
        R.m[2][2] =  cy * cp;

        return R;
    }

    public RotationMatrix transpose() {
        RotationMatrix T = new RotationMatrix();
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                T.m[i][j] = m[j][i];
        return T;
    }

    public RotationMatrix multiply(RotationMatrix other) {
        RotationMatrix R = new RotationMatrix();
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                for (int k = 0; k < 3; k++)
                    R.m[i][j] += m[i][k] * other.m[k][j];
        return R;
    }

    public double getYaw() {
        // yaw about Z
        return Math.atan2(m[0][2], m[2][2]);
    }
}