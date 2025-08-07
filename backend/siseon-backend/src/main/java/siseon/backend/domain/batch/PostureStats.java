package siseon.backend.domain.batch;

import jakarta.persistence.*;
import lombok.*;
import siseon.backend.config.JsonConverter;

import java.time.LocalDateTime;
import java.util.Map;

@Entity
@Table(name = "posture_stats")
@Getter @Setter
@NoArgsConstructor
@AllArgsConstructor
@Builder
public class PostureStats {

    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    private Long id;

    private Long profileId;

    @Column(name = "monitor_coord", columnDefinition = "JSON", nullable = false)
    @Convert(converter = JsonConverter.class)
    private Map<String, Object> monitorCoord;

    @Column(name = "user_coord", columnDefinition = "JSON")
    @Convert(converter = JsonConverter.class)
    private Map<String, Object> userCoord;

    private LocalDateTime startAt;
    private LocalDateTime endAt;

    private int durationSeconds;
    private int slotIndex;

}
