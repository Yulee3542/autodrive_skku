"""공유 흰색 판정 — BEV/ROI 내 Otsu 적응 임계 마스크.

대회 규격상 차선·정지선·주차선·장애물 차량이 전부 흰색이라 검출기들이 같은
"흰색 판정"을 공유한다. 예전에는 config.WHITE_HSV의 **고정** 임계(v_min=180)를
썼는데, 실내 트랙 조명에서 흰 선의 V가 150 근처로 내려가면 전부 놓치는 실패가
실측 확인됐다(2026-07-23: 같은 dim 프레임에서 stop_line/obstacle 모두 미검출).
lane_follow가 먼저 "BEV 안에서 Otsu 적응 임계"로 전환해 해결했고(팀 저장소
lane_detector_node.py의 해결책 이식), 이 모듈은 그 검증된 로직을 나머지
검출기들이 공유하도록 뽑아낸 것이다.

⚠️ Otsu의 함정: Otsu는 히스토그램을 "밝은 쪽/어두운 쪽"으로 **무조건** 가른다.
ROI에 흰 선이 아예 없으면 노면 자체를 반으로 갈라 절반을 흰색으로 만들어버려
정지선/장애물 오검출(= 트랙 한복판 급정지, 헛 차선변경)을 낸다. 그래서
`max_white_frac`으로 "흰 픽셀이 이만큼이나 나오면 그건 선이 아니라 노면을
가른 것"을 걸러 빈 마스크를 반환한다. 고정 임계에는 없던 위험이라 반드시 필요.

ROS 비의존. 셀프테스트: python3 -m autodrive_skku_ros.vision --selftest
"""
try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = None
    np = None

# ⚠️ 가드의 주력: Otsu가 가른 두 무리의 평균 밝기 차(대비). 진짜 흰 선이 있으면
# 밝은 무리와 어두운 무리가 확연히 떨어지지만(예: 선 150 vs 노면 90 → 60),
# 균일한 노면을 억지로 가르면 두 평균이 붙어 있다(실측: 130±12 노면 → 약 6).
# 비율만으로는 못 거른다 — 균일 노면을 가르면 정확히 절반(≈0.46)이 나와서
# "장애물 22% / 정지선 14%"와 비율만으로는 구분되지 않기 때문(실측 확인).
MIN_CONTRAST = 35

# 비율 가드는 보조 백스톱(흰 벽이 ROI를 가득 채우는 등). 대비 가드를 통과한
# 뒤에도 이만큼 넘게 흰색이면 선이 아니라고 본다.
MAX_WHITE_FRAC = 0.6

# Otsu가 너무 낮게 잡는 것을 막는 기본 바닥값. config.WHITE_HSV의 v_min(180)은
# "고정 임계로 믿을 수 있는 값"이라 바닥값으로 쓰면 Otsu가 잡을 수 있는 어두운
# 조명까지 도로 걸러버린다 — 훨씬 낮아야 한다(팀원 실측값 110 차용).
DEFAULT_V_MIN_FLOOR = 110


def white_mask(bgr, s_max, v_min_floor=DEFAULT_V_MIN_FLOOR,
               min_contrast=MIN_CONTRAST, max_white_frac=MAX_WHITE_FRAC,
               open_ksize=3, debug=None):
    """BGR ROI → 흰색 이진 마스크(0/255, uint8). cv2 미설치/None이면 None.

    임계 = max(Otsu(V), v_min_floor) 이고, 동시에 채도 S <= s_max 인 픽셀만
    흰색으로 인정한다(무채색 게이트 — 조명 밝기와 무관하므로 공유해도 안전).

    가드(모듈 docstring의 ⚠️ 참고) 둘 중 하나라도 걸리면 빈 마스크를 반환한다:
      min_contrast   — Otsu 두 무리의 평균 밝기 차가 이보다 작으면 "선 없음"
      max_white_frac — 그러고도 흰 비율이 이보다 크면 "선 없음"
    각각 None을 주면 그 가드만 끈다.

    debug: dict를 넘기면 otsu/thr/contrast/white_frac/rejected를 채운다.
    """
    if cv2 is None or np is None or bgr is None or bgr.size == 0:
        return None
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    v, s = hsv[:, :, 2], hsv[:, :, 1]
    otsu_thr, _ = cv2.threshold(v, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    thr = max(float(otsu_thr), float(v_min_floor))
    mask = ((v >= thr) & (s <= s_max)).astype(np.uint8) * 255
    if open_ksize:
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,
                                np.ones((open_ksize, open_ksize), np.uint8))

    # 대비: Otsu 임계 기준 두 무리의 평균 밝기 차 (마스크가 아니라 Otsu 자체 기준 —
    # v_min_floor/채도 게이트가 한쪽을 비워도 대비 판정은 그대로 유효해야 하므로).
    # ⚠️ 분할은 OpenCV THRESH_BINARY 규약(dst = src > thresh)과 **같은 부등호**를
    # 써야 한다. `>=`로 쓰면 Otsu가 어두운 무리의 값 자체를 임계로 돌려줄 때
    # (예: 값이 90/150 두 개뿐이면 thr=90) 전부 밝은 무리로 들어가 어두운 무리가
    # 비고 대비가 0이 돼, 진짜 선이 있는 프레임을 통째로 기각한다(실측 확인).
    hi, lo = v[v > otsu_thr], v[v <= otsu_thr]
    contrast = (float(hi.mean()) - float(lo.mean())) if hi.size and lo.size else 0.0
    white_frac = (mask > 0).sum() / float(mask.size) if mask.size else 0.0
    rejected = None
    if min_contrast is not None and contrast < min_contrast:
        rejected = "contrast"
    elif max_white_frac is not None and white_frac > max_white_frac:
        rejected = "white_frac"
    if debug is not None:
        debug.update(otsu=float(otsu_thr), thr=thr, contrast=contrast,
                     white_frac=white_frac, rejected=rejected)
    if rejected:
        return np.zeros_like(mask)
    return mask


def _selftest():
    ok = True

    def check(name, cond):
        nonlocal ok
        print(f"  [{'OK' if cond else 'X '}] {name}")
        ok = ok and bool(cond)

    print("== vision.white_mask ==")
    if cv2 is None:
        print("  [OK] cv2 미설치 — 스킵")
        return True

    # (1) 밝은 조명: 흰 선 검출 (고정임계로도 되던 경우 — 회귀 없음 확인)
    bright = np.zeros((100, 200, 3), np.uint8)
    bright[:, 90:110] = 255
    m = white_mask(bright, s_max=60)
    check("밝은 흰 선(V=255) 검출", m is not None and (m > 0).sum() > 0)

    # (2) 어두운 조명: 예전 고정임계(180)라면 전멸했을 케이스
    dim = np.full((100, 200, 3), 90, np.uint8)
    dim[:, 90:110] = 150
    m2 = white_mask(dim, s_max=60)
    check("어두운 흰 선(V=150 < 고정임계 180) 검출 — 적응 임계의 핵심",
          m2 is not None and (m2 > 0).sum() > 0)
    check("검출 위치가 실제 선 위치(90~110)와 일치",
          m2 is not None and m2[:, 90:110].mean() > m2[:, :80].mean())

    # (3) ⚠️ 가드: 흰 선이 없는 균일 노면 — Otsu가 노면을 갈라도 오검출 금지.
    # 이게 없으면 트랙 한복판에서 정지선 오검출로 급정지한다(비율 가드만으로는
    # 0.46이 나와 못 걸렀음 — 대비 가드가 필요했던 실제 이유).
    flat = np.full((100, 200, 3), 130, np.uint8)
    flat = cv2.add(flat, np.random.RandomState(0).randint(
        0, 12, flat.shape).astype(np.uint8))  # 약한 노이즈만
    dbg3 = {}
    m3 = white_mask(flat, s_max=60, debug=dbg3)
    frac = 0.0 if m3 is None else (m3 > 0).sum() / float(m3.size)
    check(f"흰 선 없는 노면 → 오검출 억제 (흰 비율 {frac:.2f}, 대비 {dbg3['contrast']:.1f}, "
          f"기각사유={dbg3['rejected']})", frac == 0.0 and dbg3["rejected"] == "contrast")

    # (4) 가드 해제 시에는 통과 — 가드가 실제로 동작 중이라는 반증
    m4 = white_mask(flat, s_max=60, min_contrast=None, max_white_frac=None)
    frac4 = 0.0 if m4 is None else (m4 > 0).sum() / float(m4.size)
    check(f"가드 해제 시엔 통과 (흰 비율 {frac4:.2f}) — 가드가 실효 중임을 반증",
          frac4 > 0.3)

    # (4b) 진짜 선이 있는 dim 프레임은 대비 가드를 통과해야 한다(과잉 차단 방지)
    dbg4 = {}
    white_mask(dim, s_max=60, debug=dbg4)
    check(f"dim이지만 진짜 선 있는 프레임은 통과 (대비 {dbg4['contrast']:.1f} >= {MIN_CONTRAST})",
          dbg4["rejected"] is None)

    # (5) 채도 게이트: 밝지만 유채색(파랑)은 흰색이 아님
    colored = np.zeros((100, 200, 3), np.uint8)
    colored[:, 90:110] = (255, 0, 0)  # BGR 파랑, V=255 S=255
    m5 = white_mask(colored, s_max=60)
    check("밝아도 채도 높으면(파랑) 흰색 아님", m5 is None or (m5 > 0).sum() == 0)

    # (6) 안전 반환
    check("None 프레임 → None", white_mask(None, 60) is None)
    return ok


if __name__ == "__main__":
    import sys
    if "--selftest" in sys.argv:
        sys.exit(0 if _selftest() else 1)
    print(__doc__)
