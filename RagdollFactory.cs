namespace DeterministicPhysics2D;

public enum RagdollPart { None, Head, Torso, Pelvis, Limb }

public static class RagdollFactory
{
    public const int BodyCount = 11, JointCount = 10;
    // Position is the center of the chest. Capacity is checked before any mutation.
    public static bool TrySpawn(PhysicsWorld world, Vec2 position)
    {
        if (!float.IsFinite(position.X) || !float.IsFinite(position.Y)) throw new ArgumentException("Invalid ragdoll position.");
        if (world.Bodies.Length + BodyCount > PhysicsWorld.MaxBodies || world.Joints.Length + JointCount > PhysicsWorld.MaxJoints) return false;
        int group = world.AllocateCollisionGroup();
        RigidBody head = Add(world, RigidBody.Circle(position + new Vec2(0, -39), 13, 0.8f), group, RagdollPart.Head);
        RigidBody chest = Add(world, RigidBody.Box(position, 28, 46, 3), group, RagdollPart.Torso);
        RigidBody pelvis = Add(world, RigidBody.Box(position + new Vec2(0, 35), 26, 20, 2), group, RagdollPart.Pelvis);
        world.AddJoint(new(head, chest, position + new Vec2(0, -25), -0.55f, 0.55f));
        world.AddJoint(new(chest, pelvis, position + new Vec2(0, 24), -0.4f, 0.4f));
        for (int side = -1; side <= 1; side += 2)
        {
            Vec2 shoulder = position + new Vec2(side * 16, -16);
            Vec2 elbow = shoulder + new Vec2(side * 16, 26);
            Vec2 hand = elbow + new Vec2(side * 6, 27);
            RigidBody upperArm = Segment(world, shoulder, elbow, 10, 0.65f, group);
            RigidBody lowerArm = Segment(world, elbow, hand, 9, 0.5f, group);
            world.AddJoint(new(chest, upperArm, shoulder, -2.5f, 2.5f));
            world.AddJoint(new(upperArm, lowerArm, elbow, side < 0 ? -2.4f : -0.15f, side < 0 ? 0.15f : 2.4f));
            Vec2 hip = position + new Vec2(side * 8, 44);
            Vec2 knee = hip + new Vec2(side * 5, 34);
            Vec2 foot = knee + new Vec2(side * 2, 34);
            RigidBody thigh = Segment(world, hip, knee, 12, 1.1f, group);
            RigidBody shin = Segment(world, knee, foot, 10, 0.8f, group);
            world.AddJoint(new(pelvis, thigh, hip, -1.1f, 1.1f));
            world.AddJoint(new(thigh, shin, knee, side < 0 ? -0.1f : -2.3f, side < 0 ? 2.3f : 0.1f));
        }
        return true;
    }
    private static RigidBody Add(PhysicsWorld world, RigidBody body, int group, RagdollPart part)
    {
        body.CollisionGroup = group; body.RagdollPart = part;
        body.Restitution = 0; body.StaticFriction = 0.7f; body.DynamicFriction = 0.5f;
        return world.Add(body);
    }
    private static RigidBody Segment(PhysicsWorld world, Vec2 start, Vec2 end, float width, float mass, int group)
    {
        Vec2 delta = end - start;
        return Add(world, RigidBody.Box((start + end) * 0.5f, width, delta.Length, mass, MathF.Atan2(-delta.X, delta.Y)), group, RagdollPart.Limb);
    }
}
