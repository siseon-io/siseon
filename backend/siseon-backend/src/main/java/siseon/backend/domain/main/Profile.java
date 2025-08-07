package siseon.backend.domain.main;

import jakarta.persistence.*;
import jakarta.validation.constraints.NotNull;
import lombok.*;
import org.hibernate.annotations.CreationTimestamp;
import org.hibernate.annotations.UpdateTimestamp;
import siseon.backend.config.JsonConverter;

import java.time.LocalDate;
import java.time.LocalDateTime;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

@Entity
@Table(name = "profile")
@Getter
@Setter
@NoArgsConstructor
@AllArgsConstructor
@Builder
public class Profile {

    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    @Column(name = "profile_id")
    private Long id;

    @NotNull
    @Column(nullable = false, length = 100)
    private String name;

    @NotNull
    @Column(nullable = false)
    private LocalDate birthDate;

    @NotNull
    @Column(nullable = false)
    private Float height;

    @NotNull
    @Column(name = "left_vision", nullable = false)
    private Float leftVision;

    @NotNull
    @Column(name = "right_vision", nullable = false)
    private Float rightVision;

    @Column(name = "image_url", length = 255)
    private String imageUrl;

    @NotNull
    @ManyToOne(fetch = FetchType.LAZY)
    @JoinColumn(name = "user_id", nullable = false)
    private User user;

    @Lob
    @Convert(converter = JsonConverter.class)
    @Column(columnDefinition = "JSON")
    private Map<String, Object> settings;

    @Column(name = "fcm_token", length = 512)
    private String fcmToken;

    @Builder.Default
    @OneToMany(mappedBy = "profile", cascade = CascadeType.ALL, orphanRemoval = true)
    private List<Preset> presets = new ArrayList<>();

    @OneToOne(mappedBy = "profileId", cascade = CascadeType.ALL, orphanRemoval = true, fetch = FetchType.LAZY)
    private Device device;

    @CreationTimestamp
    @Column(name = "created_at", nullable = false, updatable = false)
    private LocalDateTime createdAt;

    @UpdateTimestamp
    @Column(name = "updated_at", nullable = false)
    private LocalDateTime updatedAt;

    public void addPreset(Preset preset) {
        presets.add(preset);
        preset.setProfile(this);
    }

    public void removePreset(Preset preset) {
        presets.remove(preset);
        preset.setProfile(null);
    }

    public void setDevice(Device device) {
        this.device = device;
        if (device != null) device.setProfileId(this);
    }

    public void removeDevice() {
        if (this.device != null) {
            this.device.setProfileId(null);
            this.device = null;
        }
    }
}
