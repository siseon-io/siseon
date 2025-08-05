package siseon.backend.domain.batch;

import jakarta.persistence.*;
import lombok.*;
import java.time.LocalDateTime;

@Entity
@Table(name = "preset_suggestion")
@Getter @Setter @NoArgsConstructor
public class PresetSuggestion {
    @Id @GeneratedValue(strategy = GenerationType.IDENTITY)
    private Long id;

    private Long profileId;
    private double avgX, avgY, avgZ;
    private int durationSeconds;
    private LocalDateTime startAt, endAt;
    private boolean satisfied = false;

    @Column(nullable = false, updatable = false)
    private LocalDateTime suggestedAt = LocalDateTime.now();
}