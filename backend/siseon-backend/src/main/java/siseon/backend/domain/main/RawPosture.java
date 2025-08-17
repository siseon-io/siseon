package siseon.backend.domain.main;

import jakarta.persistence.*;
import lombok.*;
import org.hibernate.annotations.CreationTimestamp;
import siseon.backend.config.JsonConverter;

import java.time.LocalDateTime;
import java.util.Map;

@Entity
@Table(name = "raw_postures")
@Getter
@Setter
@NoArgsConstructor
@AllArgsConstructor
@Builder
public class RawPosture {
    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    private Long id;

    private Long profileId;

    @Column(name = "user_coord", columnDefinition = "JSON", nullable = false)
    @Convert(converter = JsonConverter.class)
    private Map<String, Object> userCoord;

    @CreationTimestamp
    @Column(nullable = false, updatable = false)
    private LocalDateTime collectedAt;
}
