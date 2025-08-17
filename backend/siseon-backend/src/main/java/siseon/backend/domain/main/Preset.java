package siseon.backend.domain.main;

import jakarta.persistence.*;
import jakarta.validation.constraints.NotNull;
import lombok.*;
import org.hibernate.annotations.CreationTimestamp;
import org.hibernate.annotations.UpdateTimestamp;

import java.time.LocalDateTime;

@Entity
@Table(name = "preset")
@Getter @Setter
@NoArgsConstructor @AllArgsConstructor @Builder
public class Preset {

    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    @Column(name = "preset_id")
    private Long presetId;

    @NotNull
    @ManyToOne(fetch = FetchType.LAZY)
    @JoinColumn(name = "profile_id", nullable = false)
    private Profile profile;

    @NotNull
    @Column(nullable = false, length = 100)
    private String name;

    @NotNull @Column(name = "lefteye_x", nullable = false)
    private double lefteyeX;
    @NotNull @Column(name = "lefteye_y", nullable = false)
    private double lefteyeY;
    @NotNull @Column(name = "lefteye_z", nullable = false)
    private double lefteyeZ;

    @NotNull @Column(name = "righteye_x", nullable = false)
    private double righteyeX;
    @NotNull @Column(name = "righteye_y", nullable = false)
    private double righteyeY;
    @NotNull @Column(name = "righteye_z", nullable = false)
    private double righteyeZ;

    @CreationTimestamp
    @Column(name = "created_at", nullable = false, updatable = false)
    private LocalDateTime createdAt;

    @UpdateTimestamp
    @Column(name = "updated_at", nullable = false)
    private LocalDateTime updatedAt;
}
