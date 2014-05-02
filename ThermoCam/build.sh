THERMOCAM “= \
  ThermoCamStream \
  ThermoCamSingle \
“
for THERMOCAM in THERMOCAM; do
  make –C $THERMOCAM
done
