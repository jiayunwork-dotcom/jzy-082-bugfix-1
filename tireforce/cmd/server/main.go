package main

import (
	"log"
	"os"

	"tireforce/internal/api"
)

func main() {
	port := os.Getenv("PORT")
	if port == "" {
		port = "8080"
	}
	r := api.NewRouter()
	log.Printf("tireforce: listening on :%s", port)
	if err := r.Run(":" + port); err != nil {
		log.Fatal(err)
	}
}
