#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import glob
import os
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml


IMAGE_EXTENSIONS = ("*.png", "*.jpg", "*.jpeg", "*.bmp", "*.tif", "*.tiff")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Kalibracia Basler kamery zo sachovnicovych fotiek."
    )

    parser.add_argument(
        "--images-dir",
        default="fotky",
        help="Priecinok s fotkami sachovnice. Default: fotky",
    )

    parser.add_argument(
        "--cols",
        type=int,
        default=5,
        help="Pocet VNUTORNYCH rohov sachovnice v smere stlpcov. Pre tvoje fotky pravdepodobne 5.",
    )

    parser.add_argument(
        "--rows",
        type=int,
        default=3,
        help="Pocet VNUTORNYCH rohov sachovnice v smere riadkov. Pre tvoje fotky pravdepodobne 4.",
    )

    parser.add_argument(
        "--square-mm",
        type=float,
        default=40.0,
        help="Velkost jedneho stvorceka v mm. Zmeraj realnu hodnotu. Default: 25.0",
    )

    parser.add_argument(
        "--output",
        default="config/basler_chessboard_ros.yaml",
        help="Vystupny ROS camera_info YAML subor.",
    )

    parser.add_argument(
        "--camera-name",
        default="basler_camera",
        help="Nazov kamery v YAML subore.",
    )

    parser.add_argument(
        "--debug-dir",
        default="calib_debug",
        help="Priecinok na debug obrazky s najdenymi rohmi.",
    )

    parser.add_argument(
        "--detect-scale",
        type=float,
        default=0.5,
        help="Mierka obrazu na hladanie rohov. 0.5 je rychlejsie pre 1920x1200."
    )

    parser.add_argument(
        "--try-swapped",
        action="store_true",
        help="Skusi aj prehodene cols/rows, ak zakladny vzor nenajde rohy."
    )

    return parser.parse_args()


def collect_images(images_dir):
    images_dir = Path(images_dir).expanduser()
    files = []
    for ext in IMAGE_EXTENSIONS:
        files.extend(glob.glob(str(images_dir / ext)))
    files = sorted(files)
    return files


def make_object_points(cols, rows, square_mm):
    # Body sachovnice v realnom svete: z = 0, rozmery v mm.
    objp = np.zeros((rows * cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp *= float(square_mm)
    return objp


def detect_chessboard(gray, pattern_size, detect_scale):
    """
    Najde rohy sachovnice.
    Najprv hlada na zmensenej fotke, potom body prepocita spat do plneho rozlisenia
    a jemne ich doladi cez cornerSubPix.
    """
    cols, rows = pattern_size

    if detect_scale <= 0 or detect_scale > 1.0:
        detect_scale = 1.0

    if detect_scale < 1.0:
        small = cv2.resize(gray, None, fx=detect_scale, fy=detect_scale, interpolation=cv2.INTER_AREA)
    else:
        small = gray

    flags_sb = cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY | cv2.CALIB_CB_NORMALIZE_IMAGE

    try:
        ok, corners_small = cv2.findChessboardCornersSB(small, (cols, rows), flags=flags_sb)
    except Exception:
        ok = False
        corners_small = None

    if not ok or corners_small is None:
        return False, None

    corners_full = corners_small.astype(np.float32) / float(detect_scale)

    # Jemne doladenie rohov na plnom obraze.
    # Pri findChessboardCornersSB su uz body dobre, ale toto ich este zjemni.
    term = (
        cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
        50,
        0.001,
    )

    try:
        cv2.cornerSubPix(gray, corners_full, (7, 7), (-1, -1), term)
    except Exception:
        pass

    return True, corners_full


def reprojection_errors(objpoints, imgpoints, rvecs, tvecs, camera_matrix, dist_coeffs):
    errors = []
    total_error = 0.0
    total_points = 0

    for objp, imgp, rvec, tvec in zip(objpoints, imgpoints, rvecs, tvecs):
        projected, _ = cv2.projectPoints(objp, rvec, tvec, camera_matrix, dist_coeffs)
        err = cv2.norm(imgp, projected, cv2.NORM_L2)
        n = len(projected)
        errors.append(err / n)
        total_error += err * err
        total_points += n

    rms_like = np.sqrt(total_error / total_points) if total_points > 0 else float("nan")
    mean_error = float(np.mean(errors)) if errors else float("nan")
    return mean_error, rms_like, errors


def save_ros_camera_info_yaml(path, camera_name, width, height, camera_matrix, dist_coeffs):
    path = Path(path).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)

    # OpenCV vie vratit 1x5 alebo viac koeficientov. Pre ROS plumb_bob pouzijeme prvych 5.
    d = dist_coeffs.flatten().astype(float).tolist()
    if len(d) < 5:
        d = d + [0.0] * (5 - len(d))
    d = d[:5]

    fx = float(camera_matrix[0, 0])
    fy = float(camera_matrix[1, 1])
    cx = float(camera_matrix[0, 2])
    cy = float(camera_matrix[1, 2])

    data = {
        "image_width": int(width),
        "image_height": int(height),
        "camera_name": str(camera_name),
        "camera_matrix": {
            "rows": 3,
            "cols": 3,
            "data": [
                float(camera_matrix[0, 0]), float(camera_matrix[0, 1]), float(camera_matrix[0, 2]),
                float(camera_matrix[1, 0]), float(camera_matrix[1, 1]), float(camera_matrix[1, 2]),
                float(camera_matrix[2, 0]), float(camera_matrix[2, 1]), float(camera_matrix[2, 2]),
            ],
        },
        "distortion_model": "plumb_bob",
        "distortion_coefficients": {
            "rows": 1,
            "cols": 5,
            "data": [float(x) for x in d],
        },
        "rectification_matrix": {
            "rows": 3,
            "cols": 3,
            "data": [
                1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0,
            ],
        },
        "projection_matrix": {
            "rows": 3,
            "cols": 4,
            "data": [
                fx, 0.0, cx, 0.0,
                0.0, fy, cy, 0.0,
                0.0, 0.0, 1.0, 0.0,
            ],
        },
    }

    with open(path, "w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, sort_keys=False, allow_unicode=True)

    return path


def save_opencv_yaml(path, width, height, camera_matrix, dist_coeffs, rms, mean_error):
    path = Path(path).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)

    fs = cv2.FileStorage(str(path), cv2.FILE_STORAGE_WRITE)
    fs.write("image_width", int(width))
    fs.write("image_height", int(height))
    fs.write("camera_matrix", camera_matrix)
    fs.write("distortion_coefficients", dist_coeffs)
    fs.write("rms", float(rms))
    fs.write("mean_reprojection_error", float(mean_error))
    fs.release()


def main():
    args = parse_args()

    image_files = collect_images(args.images_dir)
    if not image_files:
        print(f"[ERROR] Nenasiel som ziadne fotky v priecinku: {args.images_dir}")
        sys.exit(1)

    debug_dir = Path(args.debug_dir).expanduser()
    accepted_dir = debug_dir / "accepted"
    rejected_dir = debug_dir / "rejected"
    accepted_dir.mkdir(parents=True, exist_ok=True)
    rejected_dir.mkdir(parents=True, exist_ok=True)

    print("================ KALIBRACIA KAMERY ================")
    print(f"Priecinok s fotkami: {Path(args.images_dir).expanduser().resolve()}")
    print(f"Pocet fotiek: {len(image_files)}")
    print(f"Vnutorne rohy sachovnice: cols={args.cols}, rows={args.rows}")
    print(f"Velkost stvorceka: {args.square_mm} mm")
    print("====================================================")

    objpoints = []
    imgpoints = []
    used_files = []
    rejected_files = []
    image_size = None

    base_pattern = (args.cols, args.rows)
    swapped_pattern = (args.rows, args.cols)

    objp_base = make_object_points(args.cols, args.rows, args.square_mm)
    objp_swapped = make_object_points(args.rows, args.cols, args.square_mm)

    for idx, file_path in enumerate(image_files, start=1):
        img = cv2.imread(file_path)
        if img is None:
            print(f"[WARN] Neviem nacitat: {file_path}")
            rejected_files.append(file_path)
            continue

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        image_size = (gray.shape[1], gray.shape[0])  # width, height

        ok, corners = detect_chessboard(gray, base_pattern, args.detect_scale)
        objp = objp_base
        used_pattern = base_pattern

        if not ok and args.try_swapped:
            ok, corners = detect_chessboard(gray, swapped_pattern, args.detect_scale)
            objp = objp_swapped
            used_pattern = swapped_pattern

        basename = Path(file_path).name

        if ok:
            objpoints.append(objp.copy())
            imgpoints.append(corners)
            used_files.append(file_path)

            drawn = img.copy()
            cv2.drawChessboardCorners(drawn, used_pattern, corners, True)
            cv2.imwrite(str(accepted_dir / basename), drawn)
            print(f"[OK] {idx:02d}/{len(image_files)} pouzita: {basename} pattern={used_pattern}")
        else:
            rejected_files.append(file_path)
            cv2.imwrite(str(rejected_dir / basename), img)
            print(f"[--] {idx:02d}/{len(image_files)} rohy nenajdene: {basename}")

    with open(debug_dir / "used_images.txt", "w", encoding="utf-8") as f:
        for x in used_files:
            f.write(str(x) + "\n")

    with open(debug_dir / "rejected_images.txt", "w", encoding="utf-8") as f:
        for x in rejected_files:
            f.write(str(x) + "\n")

    print()
    print("================ VYSLEDOK DETEKCIE ================")
    print(f"Pouzite fotky: {len(used_files)}")
    print(f"Nepouzite fotky: {len(rejected_files)}")
    print(f"Debug priecinok: {debug_dir.resolve()}")
    print("====================================================")

    if len(used_files) < 3:
        print("[ERROR] Na kalibraciu potrebujem aspon 3 pouzitelne fotky.")
        print("Skontroluj --cols, --rows alebo nafot viac fotiek s dobre viditelnou sachovnicou.")
        sys.exit(2)

    if len(used_files) < 10:
        print("[WARN] Pouzitych je menej ako 10 fotiek. Kalibracia sa ulozi, ale odporucam nafotit 20-40 kvalitnych zaberov.")

    width, height = image_size

    # Kalibracia kamery
    rms, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        objpoints,
        imgpoints,
        image_size,
        None,
        None,
    )

    mean_error, rms_like, per_image_errors = reprojection_errors(
        objpoints,
        imgpoints,
        rvecs,
        tvecs,
        camera_matrix,
        dist_coeffs,
    )

    output_path = save_ros_camera_info_yaml(
        args.output,
        args.camera_name,
        width,
        height,
        camera_matrix,
        dist_coeffs,
    )

    opencv_path = Path(args.output).expanduser().with_name("basler_chessboard_opencv.yaml")
    save_opencv_yaml(opencv_path, width, height, camera_matrix, dist_coeffs, rms, mean_error)

    # Textovy report
    report_path = debug_dir / "calibration_report.txt"
    with open(report_path, "w", encoding="utf-8") as f:
        f.write("KALIBRACIA BASLER KAMERY\n")
        f.write("=======================\n\n")
        f.write(f"images_dir: {Path(args.images_dir).expanduser().resolve()}\n")
        f.write(f"image_width: {width}\n")
        f.write(f"image_height: {height}\n")
        f.write(f"cols: {args.cols}\n")
        f.write(f"rows: {args.rows}\n")
        f.write(f"square_mm: {args.square_mm}\n")
        f.write(f"used_images: {len(used_files)}\n")
        f.write(f"rejected_images: {len(rejected_files)}\n")
        f.write(f"rms: {rms}\n")
        f.write(f"mean_reprojection_error_px: {mean_error}\n")
        f.write(f"rms_like_reprojection_error_px: {rms_like}\n\n")
        f.write("camera_matrix:\n")
        f.write(str(camera_matrix) + "\n\n")
        f.write("dist_coeffs:\n")
        f.write(str(dist_coeffs.flatten()) + "\n")

    print()
    print("================ KALIBRACIA HOTOVA ================")
    print(f"Rozlisenie: {width} x {height}")
    print(f"RMS z calibrateCamera: {rms:.6f}")
    print(f"Priemerna reprojekcna chyba: {mean_error:.6f} px")
    print()
    print(f"ROS camera_info YAML:")
    print(f"  {output_path.resolve()}")
    print()
    print(f"OpenCV YAML:")
    print(f"  {opencv_path.resolve()}")
    print()
    print(f"Report:")
    print(f"  {report_path.resolve()}")
    print("====================================================")

    print()
    print("Do sync configu potom pouzi napriklad:")
    print(f"    calibration_file: \"{output_path.resolve()}\"")
    print("    camera_info_topic: \"/basler/camera_info\"")
    print("    publish_camera_info: true")


if __name__ == "__main__":
    main()
