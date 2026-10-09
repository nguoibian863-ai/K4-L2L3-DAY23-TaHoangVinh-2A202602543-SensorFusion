# Báo cáo bài nộp — Day 23 Sensor Fusion Lab

> Điền file này rồi commit. Cách nộp: [hướng dẫn nộp](../SUBMISSION.md).

## Thông tin học viên

- Họ tên: Tạ Hoàng Vinh
- MSSV: 2A202602543
- Email: nguoibian863@gmail.com
- Link repo (fork): https://github.com/nguoibian863-ai/K4-L2L3-DAY23-TaHoangVinh-2A202602543-SensorFusion
- Commit hash nộp (`git rev-parse HEAD`): PENDING_FINAL_COMMIT

## Tóm tắt kết quả

- `fusion_mode` (bắt buộc `compare`), `frames`, `segment`, `seed`: fusion_mode = compare, frames = [0, 198], segment = training_segment-1005081002024129653_5313_150_5333_150_with_camera_labels.tfrecord, seed = 0
- `detection.precision`, `detection.recall`, `detection.tp/fp/fn`: precision = 0.9700934579439252, recall = 0.7004048582995951, tp = 519, fp = 16, fn = 222
- `tracking.lidar.rmse`, `matches`, `sum_sq_err`, `ghost_track_frames`, `missed_gt_frames`, `mean_confirmed_tracks`: rmse = 0.15032268781360134, matches = 502, sum_sq_err = 11.343649056695735, ghost_track_frames = 0, missed_gt_frames = 239, mean_confirmed_tracks = 2.522613065326633
- `tracking.fused.rmse`, `matches`, `sum_sq_err`, `ghost_track_frames`, `missed_gt_frames`, `mean_confirmed_tracks`: rmse = 0.1358667883353908, matches = 502, sum_sq_err = 9.26681165463209, ghost_track_frames = 0, missed_gt_frames = 239, mean_confirmed_tracks = 2.522613065326633
- Giải thích khác biệt hai mode, đọc RMSE cùng số ghép và ghost/miss:
  1. Số lượng ghép và quản lý track: Cả hai mode (lidar và fused) có cùng số lượng track được confirmed và số lượng cặp ghép thành công (matches = 502 qua 198 frames), số ghost track frames bằng 0 (ghost_track_frames = 0, tức precision_track = 1.0), số missed ground-truth bằng nhau (missed_gt_frames = 239), và trung bình số confirmed tracks/frame bằng nhau (2.5226). Điều này hoàn toàn khớp với thiết kế track-then-fuse: chỉ có lượt LiDAR quyết định sự tồn tại của track (khởi tạo, tính điểm hit/miss trong FOV, xác nhận và xoá track), còn camera hoàn toàn không can thiệp vào lifecycle hay score của track.
  2. Độ chính xác vị trí (RMSE): Ở mode fused, RMSE giảm từ 0.1503 m (ở lidar) xuống 0.1359 m (giảm -0.0145 m, thỏa mãn xuất sắc tiêu chí rmse_fused - rmse_lidar <= 0.05 m và RMSE <= 0.45 m của rubric). Tổng bình phương sai số sum_sq_err giảm từ 11.3436 m^2 xuống 9.2668 m^2. Nhờ việc bổ sung phép đo góc nhìn 2D pixel từ camera FRONT qua hàm h(x) phi tuyến tính, bộ lọc EKF được cập nhật thêm thông tin phương vị chính xác, giúp ước lượng vị trí 3D mượt hơn và giảm thiểu sai số lệch tâm của các bounding box 3D từ LiDAR detector.

Chạy từ root repo:

```bash
fusion-run-lab --config student/config/paths.yaml --fusion compare --seed 0
```

`rmse = sqrt(sum_sq_err/matches)` trên vị trí 3D của confirmed tracks ghép
một-một với GT xe trong cửa sổ BEV, gate XY **2.0 m**; `null` nếu không có cặp.
Camera dùng tâm hộp 2D ground-truth FRONT có nhiễu seeded, **không** dùng camera
detector. Kết quả này không đo hiệu quả một perception system độc lập với GT.

`grade_run.log` là JSONL, mỗi `(mode,frame)` đúng một record với các trường:
`mode`, `frame`, `det_tp`, `det_fp`, `det_fn`, `valid_gt`, `confirmed`, `matches`,
`sum_sq_err`, `ghosts`, `misses`. Đảm bảo `matches+ghosts==confirmed` và
`matches+misses==valid_gt`; tổng/trung bình record phải khớp `metrics.json`.
File per-mode `metrics_lidar.json`, `metrics_fused.json`, `grade_run_lidar.log`,
`grade_run_fused.log` được giữ để đối chiếu.

## Giải thích ngắn (Parts E–H — tự viết)

1. Khác biệt đo lidar 3D và camera 2D trong EKF (`z`, `R`)?
- Không gian đo và vector đo z: LiDAR đo trực tiếp vị trí 3D của vật thể trong không gian (hệ toạ độ cảm biến / xe), z là toạ độ tâm 3D (x, y, z) tính bằng mét thuộc R^3. Ngược lại, camera đo vị trí 2D trên mặt phẳng ảnh, z là toạ độ điểm ảnh pixel (u, v) thuộc R^2.
- Ma trận hiệp phương sai nhiễu đo R: Với LiDAR, R (3x3) là ma trận đường chéo biểu diễn phương sai sai số khoảng cách theo các trục mét (sigma_x^2, sigma_y^2, sigma_z^2) (ví dụ 0.1^2 m^2). Với camera, R (2x2) là ma trận đường chéo biểu diễn phương sai sai số pixel ảnh (sigma_u^2, sigma_v^2) (ví dụ 5.0^2 px^2).
- Hàm đo h(x) và Jacobian H: LiDAR sử dụng phép biến đổi toạ độ affine tuyến tính với Jacobian H (3x6) đơn giản. Camera sử dụng mô hình pinhole perspective projection phi tuyến tính (u = c_i - f_i * y_s / x_s, v = c_j - f_j * z_s / x_s với x_s là độ sâu), do đó ma trận Jacobian H (2x6) phải tính bằng đạo hàm theo quy tắc chuỗi (chain-rule Jacobian: đạo hàm phép chiếu nhân ma trận xoay).

2. Vì sao cần gating Mahalanobis trước khi gán?
- Gating bằng khoảng cách Mahalanobis d^2 = gamma^T S^{-1} gamma kết hợp kiểm định phân phối chi-square (với bậc tự do bằng dim_meas) cho phép loại bỏ các đo lường ngoại lai (outliers / clutter) hoặc các đo lường không tương thích về mặt xác suất với track trước khi thực hiện thuật toán gán cặp greedy.
- Khác với khoảng cách Euclidean bỏ qua độ bất định của trạng thái và cảm biến, khoảng cách Mahalanobis chuẩn hoá phần dư đổi mới gamma = z - h(x) theo ma trận hiệp phương sai đổi mới S = H P H^T + R. Khi track có độ bất định cao (P lớn theo một hướng), elip chấp nhận sẽ dãn rộng tương ứng; khi track đã được ước lượng chính xác, cổng sẽ thu hẹp lại để tránh ghép nhầm detection từ vật thể khác hoặc nhiễu môi trường. Điều này giúp ngăn chặn việc cập nhật Kalman bị sai lệch và tránh làm phân kỳ bộ lọc.

3. Pipeline là track-then-fuse hay fuse-then-track? Chỉ ra trên log `fusion-run-lab`.
- Pipeline của lab là track-then-fuse: Hệ thống duy trì một danh sách track thống nhất qua các frame. Ở mỗi frame, tracker thực hiện dự báo (EKF predict) một lần duy nhất, sau đó lần lượt thực hiện gán đo và cập nhật EKF nối tiếp theo từng modality cảm biến: đầu tiên là AssocL (gán đo LiDAR 3D, cập nhật EKF, quản lý vòng đời track), sau đó đến AssocC (gán đo camera 2D FRONT cho các track nằm trong camera FOV và cập nhật EKF để tinh chỉnh trạng thái). Không có việc gộp dữ liệu thô cảm biến trước khi phát hiện (như fuse-then-track).
- Bằng chứng trên log `fusion-run-lab`: Trong log thực thi của từng frame (khi chạy `--fusion compare` hoặc `--fusion fused`), log thể hiện rõ chuỗi bước tuần tự: EKF predict trước trên toàn bộ track, tiếp theo là block log AssocL (xử lý gán đo LiDAR, cập nhật EKF và chạy manage_tracks), sau đó mới đến block log AssocC (kiểm tra trường nhìn camera, gán pixel 2D và gọi EKF update cho track đã khớp mà không can thiệp vòng đời track).

4. Nếu camera lệch calibration, triệu chứng gì trên innovation/residual?
- Khi camera bị lệch thông số calibration (nội tại f, c hoặc ngoại tại R, t so với hệ toạ độ xe), toạ độ dự báo h(x) trên mặt phẳng ảnh sẽ bị lệch có hệ thống (systematic bias) so với phép đo thực tế z.
- Triệu chứng:
  + Phần dư innovation gamma = z - h(x) sẽ không còn phân phối chuẩn có kỳ vọng bằng 0 (zero-mean Gaussian), mà xuất hiện độ lệch trung bình cố định (non-zero mean bias) nhất quán theo hướng bị lệch.
  + Giá trị khoảng cách Mahalanobis d^2 = gamma^T S^{-1} gamma tăng cao bất thường một cách có hệ thống, khiến nhiều phép đo camera hợp lệ bị từ chối bởi cổng kiểm định chi2 (chi2 gating rejection), dẫn đến việc camera không được gán vào track.
  + Nếu vẫn lọt qua cổng gating, innovation mang độ lệch sai này sẽ kéo giật trạng thái EKF về vị trí sai, gây xung đột giữa bước cập nhật LiDAR và camera, làm tăng phương sai sai số hoặc gây phân kỳ bộ lọc EKF.

5. Vì sao `associate_and_update(..., sensor)` cần sensor tường minh ở frame rỗng?
   Giải thích vì sao lidar quyết định score/init/delete còn camera chỉ EKF update.
- Cần sensor tường minh ở frame rỗng vì cuối hàm `associate_and_update` luôn phải gọi `manager.manage_tracks(unassigned_tracks, unassigned_meas, sensor)`. Việc quản lý vòng đời track (tính điểm miss, giảm score, xoá track) chỉ được thực hiện trong lượt LiDAR (sensor.name == "lidar"). Ở các frame LiDAR không có detection nào (meas_list rỗng), toàn bộ track trong trường nhìn LiDAR phải được ghi nhận là miss và bị trừ điểm tồn tại. Nếu không truyền sensor tường minh, bộ quản lý không thể phân biệt đây là lượt LiDAR rỗng hay lượt camera rỗng để áp dụng đúng quy tắc.
- Lý do LiDAR quyết định score/init/delete còn camera chỉ EKF update:
  + LiDAR đo trực tiếp vị trí 3D với độ sâu chính xác cao, có trường nhìn bao quát rộng/360 độ quanh xe, độc lập với điều kiện ánh sáng, là nguồn dữ liệu đáng tin cậy nhất để khởi tạo vị trí 3D và xác nhận sự tồn tại của vật thể.
  + Camera trong thiết kế bài toán chỉ có góc nhìn hẹp phía trước (FRONT FOV), đo lường 2D không có chiều sâu trực tiếp. Nếu camera được phép tạo hoặc xoá track, những xe di chuyển ra ngoài góc nhìn của camera (nhưng vẫn nằm trong tầm quan sát của LiDAR) sẽ bị xoá nhầm, hoặc các phát hiện 2D giả sẽ sinh ra track 3D ảo không kiểm soát được chiều sâu. Vì vậy, camera chỉ đóng vai trò cảm biến bổ trợ tinh chỉnh trạng thái EKF khi đối tượng đi vào vùng nhìn thấy.

6. Nêu điều kiện xác nhận, giữ confirmed sau miss, và điều kiện xóa track.
- Điều kiện xác nhận track (state = "confirmed"): Khi một track ở trạng thái initialized hoặc tentative liên tục nhận được các hit LiDAR, điểm tồn tại tăng dần với mỗi hit cộng +1/window (mặc định +1/6, bão hoà tối đa 1.0). Khi điểm số vượt ngưỡng xác nhận: score > confirmed_threshold (mặc định > 0.8), track chuyển sang trạng thái "confirmed".
- Giữ confirmed sau miss: Khi một track đã đạt trạng thái "confirmed", nếu gặp frame miss trong trường nhìn của LiDAR, điểm số bị trừ -1/window (ví dụ từ 1.0 giảm còn 5/6 = 0.833), nhưng trạng thái vẫn được bảo toàn là "confirmed" (không bị hạ cấp về tentative hay initialized), giúp track ổn định qua các lần bị che khuất tạm thời hoặc detector bỏ sót ngắn hạn.
- Điều kiện xoá track (should_delete_track trả về True khi thoả mãn ít nhất một trong các điều kiện - toán tử OR):
  1. Hiệp phương sai vị trí mặt phẳng ngang quá lớn: P[0, 0] > max_P hoặc P[1, 1] > max_P (với max_P = 3.0^2 = 9.0 m^2), xoá ngay lập tức bất kể điểm score.
  2. Với track đã confirmed: Điểm số giảm xuống dưới ngưỡng xoá: score < delete_threshold (mặc định < 0.6).
  3. Với track chưa confirmed (initialized hoặc tentative): Điểm số tụt về <= 0 (score <= 0).

## Bonus (không bắt buộc)

Không

## Khai báo sử dụng AI (bắt buộc)

- Công cụ đã dùng (ChatGPT, Copilot, Claude, …): Antigravity AI Pair Programmer (Gemini)
- Dùng cho phần nào (hàm, câu hỏi, debug): Hỗ trợ phân tích mã nguồn đề bài, đối chiếu công thức EKF và quy chuẩn Waymo dataset, giải thích các câu hỏi lý thuyết Parts E–H và tự động hoá quy trình kiểm tra pytest.
- Cách bạn đã kiểm tra lại (pytest, chạy Waymo, đối chiếu công thức): Chạy toàn bộ 128 tests trong pytest student/tests, đối chiếu kết quả với các assertions trong test_kalman.py, test_tracking_contract.py, test_tracking_regressions.py, và kiểm định tính toàn vẹn với tools/check_submission.py.

## Checklist nộp

- [x] **Part E–H** trong `workspace/` đã implement; `pytest student/tests -q` không còn `failed`/`xfailed`
- [x] Part A–D: không bắt buộc sửa (hoặc ghi chú nếu bạn đã sửa)
- [x] Lần chạy chấm điểm: `--fusion compare --seed 0`, `frame_start: 0`, `frame_end: 198`
- [x] Đã commit `student/artifacts/metrics*.json` và `student/artifacts/grade_run*.log` (không sửa tay)
- [x] Đã điền đủ file này, gồm khai báo AI
- [x] Không commit dữ liệu Waymo, weights, `paths.yaml`, API key
- [x] `python tools/check_submission.py` báo `KẾT QUẢ: SẴN SÀNG NỘP`
- [ ] Đã push và nộp link repo + commit hash trên LMS ([hướng dẫn nộp](../SUBMISSION.md))
