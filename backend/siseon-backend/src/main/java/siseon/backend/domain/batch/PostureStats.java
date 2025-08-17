package siseon.backend.domain.batch;

import jakarta.persistence.*;
import lombok.*;
import siseon.backend.config.JsonConverter;

import java.time.LocalDateTime;
import java.util.Map;

@Entity
@Table(
        name = "posture_stats",
        indexes = {
                @Index(name = "idx_posture_stats_profile_endat", columnList = "profile_id, end_at"),
                @Index(name = "idx_ps_bad_scan", columnList = "profile_id, end_at, processed_bad"),
                @Index(name = "idx_ps_good_scan", columnList = "profile_id, end_at, processed_good")
        }
)
@Getter
@Setter
@NoArgsConstructor
@AllArgsConstructor
@Builder
public class PostureStats {

    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    private Long id;

    @Column(name = "profile_id", nullable = false)
    private Long profileId;

    // ✅ le_eye 평균 좌표
    @Column(name = "lefteye_x")
    private Double lefteyeX;

    @Column(name = "lefteye_y")
    private Double lefteyeY;

    @Column(name = "lefteye_z")
    private Double lefteyeZ;

    // ✅ re_eye 평균 좌표
    @Column(name = "righteye_x")
    private Double righteyeX;

    @Column(name = "righteye_y")
    private Double righteyeY;

    @Column(name = "righteye_z")
    private Double righteyeZ;

    @Column(name = "user_coord", columnDefinition = "JSON")
    @Convert(converter = JsonConverter.class)
    private Map<String, Object> userCoord;

    @Column(name = "start_at", nullable = false)
    private LocalDateTime startAt;

    @Column(name = "end_at", nullable = false)
    private LocalDateTime endAt;

    private int durationSeconds;
    private int slotIndex;

    @Column(name = "valid_posture")
    private Boolean validPosture;

    @Column(name = "bad_reasons", columnDefinition = "JSON")
    @Convert(converter = JsonConverter.class)
    private Map<String, Object> badReasons;

    @Builder.Default
    @Column(name = "processed_bad", nullable = false)
    private boolean processedBad = false;

    @Builder.Default
    @Column(name = "processed_good", nullable = false)
    private boolean processedGood = false;
}