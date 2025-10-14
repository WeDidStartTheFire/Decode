/*
 * Copyright (c) 2023, Peter Abeles. All Rights Reserved.
 *
 * This file is part of Efficient Java Matrix Library (EJML).
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package pedroPathing.localizers;

import org.ejml.data.DMatrixRMaj;

/**
 * <p>
 * This is an interface for a discrete time Kalman filter with no control input:<br>
 * <br>
 * x<sub>k</sub> = F<sub>k</sub> x<sub>k-1</sub> + w<sub>k</sub><br>
 * z<sub>k</sub> = H<sub>k</sub> x<sub>k</sub> + v<sub>k</sub> <br>
 * <br>
 * w<sub>k</sub> ~ N(0,Q<sub>k</sub>)<br>
 * v<sub>k</sub> ~ N(0,R<sub>k</sub>)<br>
 * </p>
 *
 * @author Peter Abeles
 */
public interface KalmanFilter {
    /**
     * Specify the kinematics model of the Kalman filter. This must be called
     * first before any other functions.
     *
     * @param F State transition matrix.
     * @param Q plant noise.
     * @param H measurement projection matrix.
     */
    void configure( DMatrixRMaj F, DMatrixRMaj Q, DMatrixRMaj H );

    /**
     * The prior state estimate and covariance.
     *
     * @param x The estimated system state.
     * @param P The covariance of the estimated system state.
     */
    void setState( DMatrixRMaj x, DMatrixRMaj P );

    /**
     * Predicts the state of the system forward one time step.
     */
    void predict();

    /**
     * Updates the state provided the observation from a sensor.
     *
     * @param z Measurement.
     * @param R Measurement covariance.
     */
    void update( DMatrixRMaj z, DMatrixRMaj R );

    /**
     * Returns the current estimated state of the system.
     *
     * @return The state.
     */
    DMatrixRMaj getState();

    /**
     * Returns the estimated state's covariance matrix.
     *
     * @return The covariance.
     */
    DMatrixRMaj getCovariance();



    // -----------------------
    // Kalman helper methods
    // -----------------------

    /**
     * 9x9 F matrix matching benchmark layout:
     * state = [px, py, pθ, vx, vy, vθ, ax, ay, aθ]^T
     */
    static DMatrixRMaj createF(double T) {
        double[] a = new double[]{
                1, 0, 0, T, 0, 0, 0.5 * T * T, 0, 0,
                0, 1, 0, 0, T, 0, 0, 0.5 * T * T, 0,
                0, 0, 1, 0, 0, T, 0, 0, 0.5 * T * T,
                0, 0, 0, 1, 0, 0, T, 0, 0,
                0, 0, 0, 0, 1, 0, 0, T, 0,
                0, 0, 0, 0, 0, 1, 0, 0, T,
                0, 0, 0, 0, 0, 0, 1, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 1, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 1};

        return new DMatrixRMaj(9, 9, true, a);
    }

    /**
     * Q patterned similarly to the benchmark; var is a process variance scalar to scale terms.
     */
    static DMatrixRMaj createQ(double T, double var) {
        DMatrixRMaj Q = new DMatrixRMaj(9, 9);

        double a00 = (1.0 / 4.0) * T * T * T * T * var;
        double a01 = (1.0 / 2.0) * T * T * T * var;
        double a02 = (1.0 / 2.0) * T * T * var;
        double a11 = T * T * var;
        double a12 = T * var;

        for (int i = 0; i < 3; i++) {
            Q.set(i, i, a00);
            Q.set(i, 3 + i, a01);
            Q.set(i, 6 + i, a02);
            Q.set(3 + i, 3 + i, a11);
            Q.set(3 + i, 6 + i, a12);
            Q.set(6 + i, 6 + i, var);
        }

        // mirror lower triangle
        for (int y = 1; y < 9; y++) {
            for (int x = 0; x < y; x++) {
                Q.set(y, x, Q.get(x, y));
            }
        }

        return Q;
    }

    /**
     * Simple H that maps measurement vector [px,py,heading] to state elements 0,1,2.
     * If you plan to measure more values (e.g. velocities) increase measDim and adjust mappings.
     */
    static DMatrixRMaj createH(int measDim) {
        DMatrixRMaj H = new DMatrixRMaj(measDim, 9);
        for (int i = 0; i < Math.min(measDim, 3); i++) {
            H.set(i, i, 1.0);
        }
        return H;
    }
}