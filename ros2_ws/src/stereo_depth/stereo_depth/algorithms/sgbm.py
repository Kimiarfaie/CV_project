import cv2
import numpy as np


class SGBMDepthEstimator:
    def __init__(
        self,
        min_disparity: int = 0,
        num_disparities: int = 64,
        block_size: int = 3,
        min_depth_m: float = 0.05,
        max_depth_m: float = 20.0,
        use_gamma_preprocess: bool = True,
        gamma: float = 0.75,
    ):
        self.min_disparity = min_disparity
        self.num_disparities = num_disparities
        self.block_size = block_size
        self.min_depth_m = min_depth_m
        self.max_depth_m = max_depth_m

        self.use_gamma_preprocess = use_gamma_preprocess
        self.gamma = gamma

        self.wls_lambda = 8000.0
        self.wls_sigma_color = 2.0

        self.matcher = cv2.StereoSGBM_create(
            minDisparity=self.min_disparity,
            numDisparities=self.num_disparities,
            blockSize=self.block_size,
            P1=8 * self.block_size * self.block_size,
            P2=32 * self.block_size * self.block_size,
            disp12MaxDiff=1,
            uniquenessRatio=8,
            speckleWindowSize=100,
            speckleRange=1,
            preFilterCap=63,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
        )

        self.right_matcher = cv2.ximgproc.createRightMatcher(self.matcher)

        self.wls_filter = cv2.ximgproc.createDisparityWLSFilter(
            matcher_left=self.matcher
        )
        self.wls_filter.setLambda(self.wls_lambda)
        self.wls_filter.setSigmaColor(self.wls_sigma_color)

    def preprocess(self, left: np.ndarray, right: np.ndarray):
        if not self.use_gamma_preprocess:
            return left, right

        left_f = left.astype(np.float32) / 255.0
        right_f = right.astype(np.float32) / 255.0

        left_gamma = np.power(left_f, self.gamma)
        right_gamma = np.power(right_f, self.gamma)

        left = np.clip(left_gamma * 255.0, 0, 255).astype(np.uint8)
        right = np.clip(right_gamma * 255.0, 0, 255).astype(np.uint8)

        return left, right

    def compute_disparity(self, left: np.ndarray, right: np.ndarray) -> np.ndarray:
        disparity_left = self.matcher.compute(left, right)
        disparity_right = self.right_matcher.compute(right, left)

        filtered_disparity = self.wls_filter.filter(
            disparity_left,
            left,
            disparity_map_right=disparity_right,
        )

        return filtered_disparity.astype(np.float32) / 16.0

    def compute(
        self,
        left: np.ndarray,
        right: np.ndarray,
        fx: float,
        baseline: float,
    ) -> np.ndarray:
        left, right = self.preprocess(left, right)
        disparity = self.compute_disparity(left, right)

        depth = np.full(disparity.shape, np.nan, dtype=np.float32)
        valid = np.isfinite(disparity) & (disparity > float(self.min_disparity))

        depth[valid] = (fx * baseline) / disparity[valid]
        depth[(depth < self.min_depth_m) | (depth > self.max_depth_m)] = np.nan

        return depth