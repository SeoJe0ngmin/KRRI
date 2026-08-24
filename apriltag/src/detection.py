"""2단계 — 태그 찾기.

블로그의 이 부분에 해당한다.

    detector = apriltag.Detector()
    results, dimg = detector.detect(img_gray, return_image=True)

검출 결과 하나에는 다음이 들어 있다.
    tag_id           태그 번호. 탑재부 상단/내부를 이걸로 구분한다
    homography       3x3. 정사각형 태그가 화면에서 얼마나 찌그러졌는가
    corners          네 모서리 픽셀좌표 (반시계 방향)
    center           중심 픽셀좌표
    decision_margin  디코딩 품질. 낮으면 잘못 읽었을 수 있다
    hamming          고친 오류 비트 수. 0 이 아니면 의심해볼 만하다

자세는 여기서 나오지 않는다. homography 를 3단계(pose.py)로 넘겨야 나온다.
"""
import apriltag


# 검출 전에 살짝 흐리면 오히려 검출률이 크게 오른다.
# 실촬영(tagslam) 실측: quad_blur 0 -> 56개, 2.0 -> 84개, 4.0 -> 112개.
# 대신 자세 정확도가 아주 조금 나빠진다(같은 평판 태그들의 일치 편차 0.70도 -> 0.84도).
# 검출이 50% 늘고 정확도는 0.14도 손해라 기본으로 켜둔다.
# 노이즈가 심한 실촬영이면 3.0~4.0 까지 올려볼 만하고, 깨끗한 합성영상에서는 효과가 없다.
DEFAULT_QUAD_BLUR = 2.0


def make_detector(families="tag36h11", quad_blur=DEFAULT_QUAD_BLUR, **options):
    """검출기를 만든다.

    Args:
        families: 태그 종류. AprilTag 기본이자 블로그가 쓴 것은 tag36h11.
                  tag16h5 는 코드 배치가 성겨 오검출이 많으니 피할 것.
        quad_blur: 검출 전 가우시안 블러 세기. 0 이면 끈다. 위 주석 참고.
        **options: apriltag.DetectorOptions 에 그대로 넘어간다.
            quad_decimate  1.0 이 원본 해상도. 올리면 빨라지지만 작은 태그를 놓친다
            refine_edges   모서리를 한 번 더 다듬는다. 자세 정확도가 올라간다
    """
    if quad_blur:
        options.setdefault("quad_blur", quad_blur)
    opts = apriltag.DetectorOptions(families=families, **options)
    return apriltag.Detector(opts)


def detect(detector, img_gray, min_margin=0.0, max_hamming=0):
    """태그를 찾는다.

    Args:
        img_gray: 흑백 이미지. image_source.to_gray() 를 거친 것.
        min_margin: decision_margin 이 이보다 낮으면 버린다. 오검출 방지용.
        max_hamming: 고친 오류 비트가 이보다 많으면 버린다.
                     블로그 표에 적힌 대로, 오류를 많이 고칠수록 오검출이 급증한다.

    Returns:
        검출 결과 리스트. tag_id 오름차순.
    """
    results = detector.detect(img_gray)
    keep = [r for r in results
            if r.decision_margin >= min_margin and r.hamming <= max_hamming]
    return sorted(keep, key=lambda r: r.tag_id)


def summarize(results):
    """검출 결과를 사람이 읽을 수 있게 요약."""
    lines = []
    for r in results:
        cx, cy = r.center
        lines.append(f"id={r.tag_id:<4} 중심=({cx:7.1f},{cy:7.1f}) "
                     f"margin={r.decision_margin:6.1f} hamming={r.hamming}")
    return "\n".join(lines) if lines else "(검출 없음)"
