#ifndef SLIP_DETECTION__POSE_HPP
#define SLIP_DETECTION__POSE_HPP

namespace anomaly_detection
{   
    struct Position
    {
        double x{0.0};
        double y{0.0};
        double z{0.0};

        void reset()
        {
            x = 0.0; y = 0.0; z = 0.0;
        }
    };

        
    using Velocity = Position;
    using Orietation_xyz = Position;

    struct Orietation_xyzw
    {
        double x{0.0};
        double y{0.0};
        double z{0.0};
        double w{1.0};

        void reset()
        {
            x = 0.0; y = 0.0; z = 0.0; w = 1.0;
        }
    };

    template<typename orietation_type = Orietation_xyzw>
    struct Pose
    {
        Position position;
        orietation_type orientation;

        void reset()
        {
            position.reset(); 
            orientation.reset();
        }
    };

    struct PoseDifference
    {
        double linear{0.0};
        double angle{0.0};
    };
} // namespace anomaly_detection

#endif // SLIP_DETECTION__POSE_HPP