#!/usr/bin/env python3
import os
import cv2
import time
from pypylon import pylon


def safe_set_enum(camera, name, value):
    """
    Bezpecne nastavi enum parameter kamery, napr. TriggerMode = Off.
    Ak parameter neexistuje alebo nie je zapisovatelny, iba vypise varovanie.
    """
    try:
        node = getattr(camera, name)

        if pylon.IsWritable(node):
            node.SetValue(value)
            print(f"[OK] {name} = {value}")
        else:
            print(f"[WARN] {name} nie je zapisovatelny")

    except Exception as e:
        print(f"[WARN] Nepodarilo sa nastavit {name} = {value}: {e}")


def safe_set_float(camera, name, value):
    """
    Bezpecne nastavi ciselny parameter kamery, napr. ExposureTime = 8000.0.
    """
    try:
        node = getattr(camera, name)

        if pylon.IsWritable(node):
            min_val = node.GetMin()
            max_val = node.GetMax()

            value_clamped = max(min_val, min(max_val, value))
            node.SetValue(value_clamped)

            print(f"[OK] {name} = {value_clamped}")
        else:
            print(f"[WARN] {name} nie je zapisovatelny")

    except Exception as e:
        print(f"[WARN] Nepodarilo sa nastavit {name} = {value}: {e}")


def main():
    output_dir = "fotky"
    os.makedirs(output_dir, exist_ok=True)

    print("Hladam Basler kameru...")

    camera = pylon.InstantCamera(
        pylon.TlFactory.GetInstance().CreateFirstDevice()
    )

    camera.Open()

    print()
    print("Pripojena kamera:")
    print(camera.GetDeviceInfo().GetModelName())
    print()

    # ------------------------------------------------------------
    # Nastavenie pre rucne fotenie sachovnice
    # Trigger vypneme, aby kamera isla ako live preview.
    # Toto je len na fotenie kalibracnych obrazkov.
    # Finalny sync config bude mat trigger_mode: On.
    # ------------------------------------------------------------

    safe_set_enum(camera, "AcquisitionMode", "Continuous")

    safe_set_enum(camera, "TriggerSelector", "FrameStart")
    safe_set_enum(camera, "TriggerMode", "Off")

    safe_set_enum(camera, "ExposureAuto", "Off")
    safe_set_float(camera, "ExposureTime", 8000.0)

    safe_set_enum(camera, "GainAuto", "Off")
    safe_set_float(camera, "Gain", 0.0)

    # Niektore kamery maju BalanceWhiteAuto, niektore nie
    safe_set_enum(camera, "BalanceWhiteAuto", "Off")

    # Konverzia obrazu do OpenCV BGR formatu
    converter = pylon.ImageFormatConverter()
    converter.OutputPixelFormat = pylon.PixelType_BGR8packed
    converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

    camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)

    photo_count = 0

    print()
    print("Ovládanie:")
    print("  MEDZERNIK = ulozit fotku")
    print("  q alebo ESC = koniec")
    print()
    print(f"Fotky sa ukladaju do: {os.path.abspath(output_dir)}")
    print()

    while camera.IsGrabbing():
        grab_result = camera.RetrieveResult(
            5000,
            pylon.TimeoutHandling_ThrowException
        )

        should_exit = False

        try:
            if grab_result.GrabSucceeded():
                image = converter.Convert(grab_result)
                frame = image.GetArray()

                preview = frame.copy()

                # Zmenseny nahlad, aby sa zmestil na obrazovku
                h, w = preview.shape[:2]
                max_width = 1280

                if w > max_width:
                    scale = max_width / float(w)
                    preview = cv2.resize(
                        preview,
                        None,
                        fx=scale,
                        fy=scale,
                        interpolation=cv2.INTER_AREA
                    )

                cv2.putText(
                    preview,
                    f"Fotky: {photo_count} | SPACE = ulozit | q/ESC = koniec",
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 255, 0),
                    2
                )

                cv2.imshow("Basler kamera - fotenie sachovnice", preview)

                key = cv2.waitKey(1) & 0xFF

                # MEDZERNIK
                if key == 32:
                    photo_count += 1
                    timestamp = time.strftime("%Y%m%d_%H%M%S")
                    filename = f"sachovnica_{photo_count:03d}_{timestamp}.png"
                    filepath = os.path.join(output_dir, filename)

                    cv2.imwrite(filepath, frame)
                    print(f"[ULOZENE] {filepath}")

                # q alebo ESC
                elif key == ord("q") or key == 27:
                    should_exit = True

            else:
                print("[WARN] Grab neuspesny")

        finally:
            grab_result.Release()

        if should_exit:
            break

    camera.StopGrabbing()
    camera.Close()
    cv2.destroyAllWindows()

    print()
    print(f"Hotovo. Celkovo ulozenych fotiek: {photo_count}")
    print(f"Priecinok: {os.path.abspath(output_dir)}")


if __name__ == "__main__":
    main()
