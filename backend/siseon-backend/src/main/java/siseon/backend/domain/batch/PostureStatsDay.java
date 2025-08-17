package siseon.backend.domain.batch;

import jakarta.persistence.*;
import lombok.*;

import java.time.LocalDate;
import java.time.LocalDateTime;

@Entity
@Table(
        name = "posture_stats_day",
        uniqueConstraints = @UniqueConstraint(
                name = "uq_profile_date",
                columnNames = {"profile_id", "stat_date"}
        ),
        indexes = {
                @Index(name = "idx_profile_date", columnList = "profile_id, stat_date")
        }
)
@Getter @Setter
@NoArgsConstructor @AllArgsConstructor @Builder
public class PostureStatsDay {

    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    private Long id;

    @Column(name="profile_id", nullable=false)
    private Long profileId;

    @Column(name="stat_date", nullable=false)
    private LocalDate statDate;

    @Builder.Default
    @Column(name="good_count", nullable=false)
    private int goodCount = 0;

    @Builder.Default
    @Column(name="bad_count", nullable=false)
    private int badCount = 0;

    @Column(name="created_at", nullable=false, updatable = false)
    private LocalDateTime createdAt;

    @PrePersist
    public void prePersist() {
        if (createdAt == null) {
            createdAt = LocalDateTime.now();
        }
    }
}