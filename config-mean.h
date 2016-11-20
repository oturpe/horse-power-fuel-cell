// Configuration for measuring mean values for external period of time.
// Captures about 16 hours worth of data with readings 2 minutes apart

// How many samples to average for each reading
#define SAMPLE_COUNT 20
// How long to wait between samples
#define SAMPLE_INTERVAL 100

// How often measurements are taken. Given in units of millisecond.
#define MEASUREMENT_INTERVAL (2*60*1000)
