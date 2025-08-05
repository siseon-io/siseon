package siseon.backend.domain.main;

import jakarta.persistence.*;
import jakarta.validation.constraints.NotNull;
import lombok.*;
import org.hibernate.annotations.CreationTimestamp;
import org.hibernate.annotations.UpdateTimestamp;
import siseon.backend.config.JsonConverter;

import java.time.LocalDateTime;
import java.util.Map;

@Entity
@Table(name = "preset")
@Getter
@Setter
@NoArgsConstructor
@AllArgsConstructor
@Builder
public class Preset {

    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    @Column(name = "preset_id")
    private Long presetId;

    @NotNull
    @Column(name = "device_id", nullable = false)
    private Long deviceId;

    @NotNull
    @ManyToOne(fetch = FetchType.LAZY)
    @JoinColumn(name = "profile_id", nullable = false)
    private Profile profile;

    @NotNull
    @Column(nullable = false, length = 100)
    private String name;

    @Lob
    @NotNull
    @Convert(converter = JsonConverter.class)
    @Column(columnDefinition = "JSON", nullable = false)
    private Map<String, Object> position;

    @CreationTimestamp
    @Column(name = "created_at", nullable = false, updatable = false)
    private LocalDateTime createdAt;

    @UpdateTimestamp
    @Column(name = "updated_at", nullable = false)
    private LocalDateTime updatedAt;
}
