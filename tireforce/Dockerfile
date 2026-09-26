# syntax=docker/dockerfile:1

# ---- build stage ----
FROM golang:1.22 AS build
WORKDIR /src
COPY go.mod go.sum ./
RUN go mod download
COPY . .
RUN CGO_ENABLED=0 go test ./... \
    && CGO_ENABLED=0 go build -trimpath -ldflags="-s -w" -o /out/tireforce ./cmd/server

# ---- runtime stage (Go 1.22 base image, as required) ----
FROM golang:1.22
RUN useradd --system --uid 10001 tireforce
COPY --from=build /out/tireforce /usr/local/bin/tireforce
USER tireforce
EXPOSE 8080
ENV PORT=8080
ENTRYPOINT ["tireforce"]
