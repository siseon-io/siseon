package siseon.backend.domain.batch;

import jakarta.persistence.*;
import lombok.*;
import java.time.LocalDateTime;

@Entity
@Table(name = "posture_stats")
@Getter @Setter @NoArgsConstructor
public class PostureStats {
    @Id @GeneratedValue(strategy = GenerationType.IDENTITY)
    private Long id;

    private Long profileId;
    private double avgX, avgY, avgZ;
    private LocalDateTime startAt, endAt;
    private int durationSeconds;
    private int slotIndex;

    @Column(nullable = false, updatable = false)
    private LocalDateTime createdAt = LocalDateTime.now();
}