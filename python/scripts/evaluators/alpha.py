import os
import sys

import coverage_control as cc
import numpy as np
from coverage_control import CoverageSystem
from coverage_control import IOUtils
from coverage_control import WorldIDF

from coverage_control.algorithms import ControllerCVT
from coverage_control.algorithms import ControllerNN

np.set_printoptions(formatter={"float": "{:0.2e}".format})


class Evaluator:
    """
    Class to evaluate a single environment with multiple controllers
    """

    def __init__(
        self,
        in_config,
        env_id=0,
        eta_dual=0.1,
        T_0=100,
        dual_updater=None,
        alpha=0.8,
        normalize=False,
        obj_normalize_factor=1.0,
        mu=0.5,
        sigma=0.1
    ):
        assert isinstance(env_id, int), "env_id must be an integer"
        self.obj_normalize_factor = obj_normalize_factor

        self.config = in_config
        print(self.config["Controllers"][0]["Name"])
        self.controller_config = self.config["Controllers"][0]

        if self.controller_config["Type"] == "Learning":
            controller = ControllerNN
        else:
            controller = ControllerCVT

        self.env_config_file = IOUtils.sanitize_path(self.config["EnvironmentConfig"])
        self.cc_params = cc.Parameters(self.env_config_file)

        self.num_steps = self.config["NumSteps"]

        self.plot_map = self.config["PlotMap"]
        self.generate_video = self.config["GenerateVideo"]
        env_dir = IOUtils.sanitize_path(self.config["EnvironmentDataDir"])

        self.num_idfs = self.config["NumIDFs"]

        idfs_files = [
            env_dir + "/" + f"{env_id:04}_{i:02}.env" for i in range(self.num_idfs)
        ]

        pos_file = env_dir + "/" + f"{env_id:04}.pos"

        self.world_idfs_object = [WorldIDF(self.cc_params, file) for file in idfs_files]
        self.individual_normalization_factors = [
            obj.GetNormalizationFactor() for obj in self.world_idfs_object
        ]

        # Unnormalized idfs
        self.idfs = np.stack(
            [
                np.copy(obj.GetWorldMap()) / self.individual_normalization_factors[i]
                for i, obj in enumerate(self.world_idfs_object)
            ],
            axis=-1,
        )

        self.envs = [
            CoverageSystem(self.cc_params, self.world_idfs_object[i], pos_file)
            for i in range(self.num_idfs)
        ]

        self.env_main = CoverageSystem(
            self.cc_params, WorldIDF(self.cc_params), pos_file
        )

        self.idf_main = self.env_main.GetWorldMapMutable()
        self.idf_main[:, :] = np.zeros(self.idf_main.shape)

        self.initial_obj_values = self.compute_obj_values()

        if self.generate_video:
            self.env_main.RecordPlotData()

        self.dual_updater = dual_updater

        self.controller = controller(
            self.controller_config, self.cc_params, self.env_main
        )
        self.step_counter = 0

        self.eta_dual = eta_dual

        self.T_0 = T_0  # This is in steps not in seconds

        self.max_dual_switch_counter = 0
        self.max_dual_index = 0

        self.lambda_duals = np.array([1.0 / (self.num_idfs) for i in range(self.num_idfs)])
        # self.lambda_duals = np.array([1.0 / (self.num_idf - 1.0) for i in range(self.num_idfs)])
        print(f"Initial Lambda_dual: {self.lambda_duals}")

        # Set the real values here
        # self.alphas = np.array([1 / self.num_idfs for i in range(self.num_idfs)])
        # self.alphas = np.array([alpha for i in range(self.num_idfs)])
        self.alphas = np.random.normal(mu, sigma, self.num_idfs) * self.initial_obj_values

        self.normalize = normalize

        self.update_idf(self.lambda_duals, self.normalize)

        self.all_obj_values = np.zeros((self.num_idfs, self.num_steps // self.T_0 + 1))
        self.all_lambda_duals = np.zeros((self.num_idfs, self.num_steps // self.T_0 + 1))

    def update_idf(self, coefficients, normalize=False):
        self.idf_main[:, :] = np.tensordot(self.idfs, coefficients, axes=([-1], [0]))

        normalization_factor = 1.0 / np.max(self.idf_main)
        self.idf_main *= normalization_factor

        # Clear and update the explored IDF map
        self.env_main.ClearExploredIDF()
        exp_idf = self.env_main.GetSystemExploredIDFMapMutable()

        # Prepare coefficients for multiple use
        adjusted_coefficients = (
            normalization_factor * coefficients / self.individual_normalization_factors
        )

        exp_idfs = np.stack(
            [env.GetSystemExploredIDFMap() for env in self.envs], axis=-1
        )
        exp_idf[:, :] = np.tensordot(exp_idfs, adjusted_coefficients, axes=([-1], [0]))

        # Clear robot maps
        self.env_main.ClearRobotMaps()

        # Update each robot map

        for i in range(self.env_main.GetNumRobots()):
            robot_map_main = self.env_main.GetRobotMapMutable(i)
            robot_maps = np.stack([env.GetRobotMap(i) for env in self.envs], axis=-1)

            # Apply the adjusted coefficients directly
            robot_map_main[:, :] = np.tensordot(
                robot_maps, adjusted_coefficients, axes=([-1], [0])
            )

    def advance_state(self):
        obj_val, is_converged = self.controller.step(self.env_main)
        self.step_counter = self.step_counter + 1

        if self.generate_video and self.step_counter % 1 == 0:
            self.env_main.RecordPlotData()
            # self.env_main.PlotRobotLocalMap("./robot_maps/", 0, self.step_counter)
            # self.env_main.PlotRobotSensorView("./robot_maps/", 0, self.step_counter)
        if is_converged == False:
            robot_positions = self.env_main.GetRobotPositions()
            for env in self.envs:
                env.SetGlobalRobotPositions(robot_positions)
        is_state_updated = not is_converged
        return is_state_updated

    def compute_obj_values(self):
        obj_values = np.array(
            [
                env.GetObjectiveValue() / self.obj_normalize_factor
                for i, env in enumerate(self.envs)
            ]
        )

        return obj_values

    def evaluate(self):
        # wandb.define_metric("objective_value", summary="mean")

        # for i in range(1, self.num_idfs):
        #     wandb.define_metric(f"constraints_{i}", summary="mean")

        self.all_obj_values[:, 0] = self.compute_obj_values()
        self.all_lambda_duals[:, 0] = self.lambda_duals
        K = self.num_steps // self.T_0
        # self.lambda_duals = self.fun_dual_updater(self.dual_updater, self.lambda_duals)

        self.update_idf(self.lambda_duals, normalize=self.normalize)
        obj_values = self.compute_obj_values()
        print(
            f"{0} Objective values: {obj_values} Lambda duals: {self.lambda_duals}, self alphas: {self.alphas}"
        )

        for k in range(K):

            is_state_updated = False
            for _ in range(self.T_0):
                is_state_updated = is_state_updated or self.advance_state()
                # obj_values += self.compute_obj_values()  # This is a vector

            if is_state_updated == True:
                # obj_values /= self.T_0
                obj_values = self.compute_obj_values()
                obj_max = np.max(obj_values)
                self.lambda_duals = np.maximum(
                    self.lambda_duals
                    + self.eta_dual * (obj_values - self.alphas),
                    1e-3,
                )
                # self.lambda_duals[0] = 1.0
                # if self.dual_updater == "max_one" or self.dual_updater == "malencia":
                #     self.lambda_duals = self.compute_obj_values()
                # self.lambda_duals = self.fun_dual_updater(
                #     self.dual_updater, self.lambda_duals
                # )
                self.update_idf(self.lambda_duals, normalize=self.normalize)
            else:
                obj_values = self.all_obj_values[:, k]
                self.lambda_duals = self.all_lambda_duals[:, k]
            
            self.all_obj_values[:, k + 1] = obj_values
            self.all_lambda_duals[:, k + 1] = self.lambda_duals

            print(
                f"{(k+1) * self.T_0} Objective values: {obj_values} Lambda duals: {self.lambda_duals}, self alphas: {self.alphas}"
            )

        if self.generate_video:
            self.env_main.RenderRecordedMap("./", "video.mp4")

        return self.all_obj_values, self.all_lambda_duals, self.max_dual_switch_counter

    def fun_dual_updater(self, configs, lambdas):
        if configs is None:
            return lambdas

        if configs == "ones_first_one":
            lambdas = np.array([0 for i in range(len(lambdas))])
            lambdas[0] = 1

            return lambdas

        if configs == "ones_first_two":
            lambdas = np.array([0 for i in range(len(lambdas))])
            lambdas[0] = 1
            lambdas[1] = 1

            return lambdas

        if configs == "ones_first_three":
            lambdas = np.array([0 for i in range(len(lambdas))])
            lambdas[0] = 1
            lambdas[1] = 1
            lambdas[2] = 1

            return lambdas

        if configs == "proj_1":
            assert np.all(lambdas >= 0), "All elements of v must be non-negative"
            norm_lambdas = np.sum(lambdas)
            max_index = np.argmax(lambdas)

            if self.max_dual_index != max_index:
                self.max_dual_index = max_index
                self.max_dual_switch_counter += 1

            return lambdas / norm_lambdas

        if configs == "max_one":
            # Set max to 1 and rest to 0
            max_index = np.argmax(lambdas)
            lambdas = np.zeros(len(lambdas))
            lambdas[max_index] = 1
            print(f"max_index: {max_index}")
            print(f"lambdas: {lambdas}")

            if self.max_dual_index != max_index:
                self.max_dual_index = max_index
                self.max_dual_switch_counter += 1

            return lambdas

        if configs == "avg":
            lambdas = np.array([1.0 / self.num_idfs for i in range(self.num_idfs)])
            return lambdas

        if configs == "malencia":
            exp_obj_values = np.exp(lambdas)
            norm_exp_obj_values = np.sum(exp_obj_values)
            lambdas = exp_obj_values / norm_exp_obj_values
            return lambdas

        raise ValueError("configs not recognized")


if __name__ == "__main__":
    in_config = IOUtils.load_toml(IOUtils.sanitize_path(sys.argv[1]))
    envs = list(range(100))
    # T_0s = [25, 50, 75, 100]
    # envs = [72]
    T_0s = [15]
    eta_duals = [5]
    eval_dir = sys.argv[2]
    # Get mu from sys.argv[3] as a float
    mu = float(sys.argv[3])

    for eta_dual in eta_duals:
        for T_0 in T_0s:
            for env_id in envs:
                print("\033[92m" + f"eta_dual: {eta_dual}, T_0: {T_0}, env_id: {env_id}" + "\033[0m")
                evaluator = Evaluator(
                    in_config,
                    env_id,
                    eta_dual,
                    T_0,
                    dual_updater="proj_1",
                    alpha=0.0,
                    normalize=True,
                    obj_normalize_factor=1e10,
                    mu=mu,
                    sigma=0.1
                )
                obj_values, lambda_duals, max_dual_switch_counter = evaluator.evaluate()
                # Create a directory eval_dir/eta_dual_T_0
                res_dir = f"{eval_dir}/{eta_dual}_{T_0}"
                os.makedirs(res_dir, exist_ok=True)
                # Save obj_values_env_id.csv, lambda_duals_env_id.csv, max_dual_switch_counter_env_id.csv
                np.savetxt(f"{res_dir}/obj_values_{env_id}.csv", obj_values, delimiter=",")
                np.savetxt(
                    f"{res_dir}/lambda_duals_{env_id}.csv", lambda_duals, delimiter=","
                )
                # Save alpha
                with open(f"{res_dir}/alpha_{env_id}.csv", "w") as f:
                    f.write(str(evaluator.alphas))
                # save max_dual_switch_counter_env_id to file as an integer
                with open(f"{res_dir}/max_dual_switch_counter_{env_id}.csv", "w") as f:
                    f.write(str(max_dual_switch_counter))
