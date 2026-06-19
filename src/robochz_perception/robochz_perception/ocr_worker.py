#!/usr/bin/env python3
"""PaddleOCR 상주 워커 — venv(paddleocr 설치됨) 파이썬으로 실행된다.

perception_node(순수 ROS, paddle 미설치)가 이 스크립트를 subprocess 로 1회 띄우고,
stdin 으로 요청(JSON 한 줄)을 보내면 OCR 결과를 stdout(JSON 한 줄)으로 돌려준다.
→ paddle/cv2 의존성을 ROS 환경과 완전히 분리(프로세스 경계).

프로토콜:
  perception → worker (stdin) : {"npy": "<프레임.npy>", "png": "<저장할 .png>"}\n
  worker → perception (stdout): READY            (모델 로드 완료 1회)
                                RESULT {json}\n   (요청마다)
  응답 json: {"ok": bool, "plate": str, "conf": float, "image_path": str}

ROS 와 섞이지 않게 stdout 엔 'READY' 와 'RESULT ...' 만 출력한다(paddle 로그는 show_log=False + stderr).
"""
import sys
import json
import re

import numpy as np
import cv2
from paddleocr import PaddleOCR

# 한글 음절 + 숫자만 (KOR 밴드/볼트자국/공백 제거)
KEEP = re.compile(r"[가-힣0-9]")
# 한국 번호판 패턴 (지역번호 2~3 + 한글 1 + 4)
PLATE_RE = re.compile(r"\d{2,3}[가-힣]\d{4}")


def recognize(ocr, bgr):
    """번호판 본문 박스만 x좌표 정렬해 합치고 (plate_text, 평균conf) 반환."""
    result = ocr.ocr(bgr, cls=False)
    boxes = []
    if result and result[0]:
        for box, (text, conf) in result[0]:
            kept = "".join(KEEP.findall(text))
            if kept:                       # KOR/하이픈 등은 빈 문자열 → 제외
                boxes.append((box[0][0], kept, conf))   # (x0, 정규화텍스트, conf)
    boxes.sort(key=lambda b: b[0])         # 좌→우
    plate = "".join(b[1] for b in boxes)
    conf = sum(b[2] for b in boxes) / len(boxes) if boxes else 0.0
    # 번호판 패턴이 부분포함되면 그 부분만 채택(잡텍스트 방어)
    m = PLATE_RE.search(plate)
    if m:
        plate = m.group(0)
    return plate, conf


def main():
    # show_log=False → ppocr DEBUG 가 stdout 을 오염시키지 않음
    ocr = PaddleOCR(lang='korean', show_log=False)
    print("READY", flush=True)            # 모델 로드 완료 신호

    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        try:
            req = json.loads(line)
            arr = np.load(req["npy"])      # (H,W,3) rgb8
            bgr = arr[:, :, ::-1].copy()   # paddle/cv2 는 BGR
            plate, conf = recognize(ocr, bgr)
            image_path = ""
            if req.get("png"):
                cv2.imwrite(req["png"], bgr)   # 기록용 캡처 저장
                image_path = req["png"]
            resp = {"ok": bool(plate), "plate": plate,
                    "conf": round(float(conf), 4), "image_path": image_path}
        except Exception as e:             # 워커가 죽지 않게 에러도 응답으로
            resp = {"ok": False, "plate": "", "conf": 0.0,
                    "image_path": "", "error": str(e)}
        print("RESULT " + json.dumps(resp, ensure_ascii=False), flush=True)


if __name__ == "__main__":
    main()
