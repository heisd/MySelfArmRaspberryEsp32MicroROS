from ultralytics import YOLO


def main() -> None:
    YOLO("yolov8n.pt")


if __name__ == "__main__":
    main()
