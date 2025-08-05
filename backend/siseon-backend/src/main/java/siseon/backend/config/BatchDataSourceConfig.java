package siseon.backend.config;

import com.zaxxer.hikari.HikariDataSource;
import org.springframework.beans.factory.annotation.Qualifier;
import org.springframework.boot.autoconfigure.jdbc.DataSourceProperties;
import org.springframework.boot.context.properties.ConfigurationProperties;
import org.springframework.context.annotation.*;
import org.springframework.jdbc.datasource.DataSourceTransactionManager;
import org.springframework.transaction.PlatformTransactionManager;

import javax.sql.DataSource;

@Configuration
public class BatchDataSourceConfig {

    @Bean
    @ConfigurationProperties("spring.batch.datasource")
    public DataSourceProperties batchDataSourceProperties() {
        return new DataSourceProperties();
    }

    @Bean("batchDs")
    public DataSource batchDataSource(
            @Qualifier("batchDataSourceProperties") DataSourceProperties props) {
        return props
                .initializeDataSourceBuilder()
                .type(HikariDataSource.class)
                .build();
    }

    @Bean("batchTx")
    public PlatformTransactionManager batchTransactionManager(
            @Qualifier("batchDs") DataSource ds) {
        return new DataSourceTransactionManager(ds);
    }
}