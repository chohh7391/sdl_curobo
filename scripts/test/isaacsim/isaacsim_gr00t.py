# import gr00t

# from gr00t.data.dataset import LeRobotSingleDataset
# from gr00t.model.policy import Gr00tPolicy
# from gr00t.experiment.data_config import DATA_CONFIG_MAP
# import numpy as np

# class IsaacSimGr00t:
#     def __init__(self, env):

#         self.ckpt_path = "/home/hyunho_RCI/IsaacLab/logs/gr00t/peginhole-checkpoints/checkpoint-10000"
#         self.embodiment_tag = "new_embodiment"
#         self.data_cfg = DATA_CONFIG_MAP["franka_triple_cam"]
#         self.modality_cfg = self.data_cfg.modality_config()
#         self.modality_transform = self.data_cfg.transform()

#         self.gr00t_policy = None


# def setup_gr00t_policy(self) -> Gr00tPolicy:

#     self.gr00t_policy = Gr00tPolicy(
#         model_path=self.ckpt_path,
#         modality_config=self.modality_cfg,
#         modality_transform=self.modality_transform,
#         embodiment_tag=self.embodiment_tag,
#         denoising_steps=4,
#     )

#     print(self.gr00t_policy.model)

#     self.modality_cfg = self.gr00t_policy.modality_config

#     print(self.modality_cfg.keys())

#     for key, value in self.modality_cfg.items():
#         if isinstance(value, np.ndarray):
#             print(key, value.shape)
#         else:
#             print(key, value)

#     return self.gr00t_policy


# def process_observations(self, observations) -> dict[str, np.ndarray]:

#     pass

#     # IsaacLab Version
#     """Process the observations using the GR00T policy.

#     # Args:
#     #     gr00t_policy: The GR00T policy to use for processing observations.

#     # Returns:
#     #     A dictionary containing the processed observations.
#     # """
#     # image_fixed_1 = self.shared_dict["events"]["gr00t_observations"]["image_fixed_1"]
#     # image_fixed_2 = self.shared_dict["events"]["gr00t_observations"]["image_fixed_2"]
#     # image_wrist = self.shared_dict["events"]["gr00t_observations"]["image_wrist"]

#     # # PyTorch 텐서를 NumPy 배열로 변환하고 dtype을 np.uint8로 맞춥니다.
#     # # .cpu()는 GPU에 있는 텐서인 경우 CPU로 옮기는 역할 (필요한 경우만)
#     # # .numpy()는 PyTorch 텐서를 NumPy 배열로 변환
#     # # .astype(np.uint8)은 데이터 타입을 np.uint8로 명시적으로 설정

#     # observations = {
#     #     # PyTorch 텐서를 NumPy 배열로 변환하고, uint8 타입으로 변경 후, np.expand_dims로 차원 추가
#     #     "video.ego_view": np.expand_dims(image_wrist.cpu().numpy().astype(np.uint8), axis=1),
#     #     "video.view_1": np.expand_dims(image_fixed_1.cpu().numpy().astype(np.uint8), axis=1),
#     #     "video.view_2": np.expand_dims(image_fixed_2.cpu().numpy().astype(np.uint8), axis=1),
#     #     # state.arm은 PyTorch 텐서로 유지되어도 무방합니다. (관련 에러가 없었으므로)
#     #     "state.arm": np.expand_dims(self.obs_buf["policy"][:, 0:7].cpu().numpy().astype(np.float64), axis=1)
#     # }

#     # return observations
