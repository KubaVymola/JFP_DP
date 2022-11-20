FROM grafana/grafana:latest

COPY grafana.ini /etc/grafana/grafana.ini

RUN grafana-cli plugins install marcusolsson-csv-datasource

# ENV GF_AUTH_ANONYMOUS_ENABLED=true
# ENV GF_AUTH_ORG_ROLE=admin
# ENV GF_AUTH_DISABLE_LOGIN_FORM=true
ENV GF_PLUGIN_ALLOW_LOCAL_MODE=true
