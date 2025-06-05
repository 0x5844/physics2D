package main

import (
	"context"
	"encoding/json"
	"flag"
	"fmt"
	"io"
	"log"
	"math"
	"math/rand"
	"os"
	"os/signal"
	"runtime"
	"runtime/pprof"
	"strings"
	"sync"
	"sync/atomic"
	"syscall"
	"time"
	"unsafe"
)

// Build information (set by build script)
var (
	Version   = "dev"
	BuildTime = "unknown"
	GoVersion = "unknown"
)

// ==================== MATHEMATICAL FOUNDATION ====================

type Vector2D struct {
	X, Y float64
}

func NewVector2D(x, y float64) Vector2D {
	return Vector2D{X: x, Y: y}
}

func (v1 Vector2D) Add(v2 Vector2D) Vector2D {
	return Vector2D{X: v1.X + v2.X, Y: v1.Y + v2.Y}
}

func (v1 Vector2D) Sub(v2 Vector2D) Vector2D {
	return Vector2D{X: v1.X - v2.X, Y: v1.Y - v2.Y}
}

func (v Vector2D) Scale(factor float64) Vector2D {
	return Vector2D{X: v.X * factor, Y: v.Y * factor}
}

func (v Vector2D) Dot(other Vector2D) float64 {
	return v.X*other.X + v.Y*other.Y
}

func (v Vector2D) Cross(other Vector2D) float64 {
	return v.X*other.Y - v.Y*other.X
}

func (v Vector2D) Magnitude() float64 {
	return math.Sqrt(v.X*v.X + v.Y*v.Y)
}

func (v Vector2D) MagnitudeSquared() float64 {
	return v.X*v.X + v.Y*v.Y
}

func (v Vector2D) Normalize() Vector2D {
	mag := v.Magnitude()
	if mag == 0 {
		return Vector2D{}
	}
	invMag := 1.0 / mag
	return Vector2D{X: v.X * invMag, Y: v.Y * invMag}
}

func (v Vector2D) Distance(other Vector2D) float64 {
	return v.Sub(other).Magnitude()
}

func (v Vector2D) DistanceSquared(other Vector2D) float64 {
	return v.Sub(other).MagnitudeSquared()
}

func fastInvSqrt(x float32) float32 {
	const threeHalfs = 1.5
	x2 := x * 0.5
	y := x
	i := *(*int32)(unsafe.Pointer(&y))
	i = 0x5f3759df - (i >> 1)
	y = *(*float32)(unsafe.Pointer(&i))
	y = y * (threeHalfs - (x2 * y * y))
	return y
}

// ==================== COLLISION TYPES ====================

type CollisionManifold struct {
	BodyA, BodyB     *RigidBody
	Normal           Vector2D
	Penetration      float64
	ContactPoints    []Vector2D
	RelativeVelocity Vector2D
	Impulse          float64
	Friction         float64
	Restitution      float64
}

type AABB struct {
	Min, Max Vector2D
}

func NewAABB(min, max Vector2D) AABB {
	return AABB{Min: min, Max: max}
}

func (aabb AABB) Overlaps(other AABB) bool {
	return aabb.Min.X <= other.Max.X && aabb.Max.X >= other.Min.X &&
		aabb.Min.Y <= other.Max.Y && aabb.Max.Y >= other.Min.Y
}

func (aabb AABB) Contains(point Vector2D) bool {
	return point.X >= aabb.Min.X && point.X <= aabb.Max.X &&
		point.Y >= aabb.Min.Y && point.Y <= aabb.Max.Y
}

func (aabb AABB) Area() float64 {
	width := aabb.Max.X - aabb.Min.X
	height := aabb.Max.Y - aabb.Min.Y
	return width * height
}

func (aabb AABB) Center() Vector2D {
	return Vector2D{
		X: (aabb.Min.X + aabb.Max.X) * 0.5,
		Y: (aabb.Min.Y + aabb.Max.Y) * 0.5,
	}
}

func (aabb AABB) Expand(margin float64) AABB {
	return AABB{
		Min: Vector2D{X: aabb.Min.X - margin, Y: aabb.Min.Y - margin},
		Max: Vector2D{X: aabb.Max.X + margin, Y: aabb.Max.Y + margin},
	}
}

// ==================== SHAPE INTERFACE ====================

type Shape interface {
	GetAABB(position Vector2D) AABB
	GetMomentOfInertia(mass float64) float64
	TestPoint(position, point Vector2D) bool
	GetType() ShapeType
}

type ShapeType int

const (
	ShapeTypeCircle ShapeType = iota
	ShapeTypeBox
	ShapeTypePolygon
)

type CircleShape struct {
	Radius float64
}

func NewCircleShape(radius float64) *CircleShape {
	return &CircleShape{Radius: radius}
}

func (c *CircleShape) GetAABB(position Vector2D) AABB {
	return AABB{
		Min: Vector2D{X: position.X - c.Radius, Y: position.Y - c.Radius},
		Max: Vector2D{X: position.X + c.Radius, Y: position.Y + c.Radius},
	}
}

func (c *CircleShape) GetMomentOfInertia(mass float64) float64 {
	return 0.5 * mass * c.Radius * c.Radius
}

func (c *CircleShape) TestPoint(position, point Vector2D) bool {
	return position.DistanceSquared(point) <= c.Radius*c.Radius
}

func (c *CircleShape) GetType() ShapeType {
	return ShapeTypeCircle
}

type BoxShape struct {
	Width, Height float64
}

func NewBoxShape(width, height float64) *BoxShape {
	return &BoxShape{Width: width, Height: height}
}

func (b *BoxShape) GetAABB(position Vector2D) AABB {
	halfW, halfH := b.Width*0.5, b.Height*0.5
	return AABB{
		Min: Vector2D{X: position.X - halfW, Y: position.Y - halfH},
		Max: Vector2D{X: position.X + halfW, Y: position.Y + halfH},
	}
}

func (b *BoxShape) GetMomentOfInertia(mass float64) float64 {
	return mass * (b.Width*b.Width + b.Height*b.Height) / 12.0
}

func (b *BoxShape) TestPoint(position, point Vector2D) bool {
	aabb := b.GetAABB(position)
	return aabb.Contains(point)
}

func (b *BoxShape) GetType() ShapeType {
	return ShapeTypeBox
}

// ==================== OBJECT POOLING SYSTEM ====================

type ObjectPool struct {
	manifoldPool sync.Pool
	vector2DPool sync.Pool
	aabbPool     sync.Pool
}

func NewObjectPool() *ObjectPool {
	return &ObjectPool{
		manifoldPool: sync.Pool{
			New: func() interface{} {
				return &CollisionManifold{
					ContactPoints: make([]Vector2D, 0, 4),
				}
			},
		},
		vector2DPool: sync.Pool{
			New: func() interface{} {
				return make([]Vector2D, 0, 4)
			},
		},
		aabbPool: sync.Pool{
			New: func() interface{} {
				return &AABB{}
			},
		},
	}
}

func (op *ObjectPool) GetManifold() *CollisionManifold {
	manifold := op.manifoldPool.Get().(*CollisionManifold)
	manifold.ContactPoints = manifold.ContactPoints[:0]
	manifold.Impulse = 0
	manifold.Penetration = 0
	return manifold
}

func (op *ObjectPool) PutManifold(manifold *CollisionManifold) {
	op.manifoldPool.Put(manifold)
}

// ==================== ENHANCED RIGID BODY ====================

const (
	SleepLinearTolerance  = 0.01
	SleepAngularTolerance = 0.017453292519943295
	SleepTime             = 0.5

	BodyStateAwake = iota
	BodyStateSleeping
)

type RigidBody struct {
	position     Vector2D
	velocity     Vector2D
	force        Vector2D
	acceleration Vector2D

	angle           float64
	angularVelocity float64
	torque          float64
	orientation     float64
	rotation        float64
	_padding1       float64

	mass        float64
	invMass     float64
	inertia     float64
	invInertia  float64
	restitution float64
	friction    float64
	damping     float64
	_padding2   float64

	shape     Shape
	isStatic  bool
	sleepTime float64
	state     int32
	id        uint64
	_padding3 [3]byte

	mu sync.RWMutex

	lastPosition Vector2D
	lastAngle    float64
}

func NewRigidBody(mass float64, position Vector2D, shape Shape) *RigidBody {
	invMass := 0.0
	inertia := 0.0
	invInertia := 0.0

	if mass > 0 {
		invMass = 1.0 / mass
		inertia = shape.GetMomentOfInertia(mass)
		if inertia > 0 {
			invInertia = 1.0 / inertia
		}
	}

	return &RigidBody{
		position:     position,
		lastPosition: position,
		mass:         mass,
		invMass:      invMass,
		inertia:      inertia,
		invInertia:   invInertia,
		restitution:  0.8,
		friction:     0.3,
		damping:      0.999,
		shape:        shape,
		isStatic:     mass == 0,
		state:        BodyStateAwake,
		id:           uint64(rand.Int63()),
		sleepTime:    0,
	}
}

func (rb *RigidBody) ApplyForce(force Vector2D) {
	if rb.isStatic || rb.state == BodyStateSleeping {
		return
	}
	rb.mu.Lock()
	rb.force = rb.force.Add(force)
	rb.mu.Unlock()
	rb.WakeUp()
}

func (rb *RigidBody) ApplyImpulse(impulse Vector2D, contactPoint Vector2D) {
	if rb.isStatic {
		return
	}
	rb.mu.Lock()
	rb.velocity = rb.velocity.Add(impulse.Scale(rb.invMass))
	r := contactPoint.Sub(rb.position)
	angularImpulse := r.Cross(impulse)
	rb.angularVelocity += angularImpulse * rb.invInertia
	rb.mu.Unlock()
	rb.WakeUp()
}

func (rb *RigidBody) GetPosition() Vector2D {
	rb.mu.RLock()
	pos := rb.position
	rb.mu.RUnlock()
	return pos
}

func (rb *RigidBody) GetVelocity() Vector2D {
	rb.mu.RLock()
	vel := rb.velocity
	rb.mu.RUnlock()
	return vel
}

func (rb *RigidBody) GetAABB() AABB {
	rb.mu.RLock()
	aabb := rb.shape.GetAABB(rb.position)
	rb.mu.RUnlock()
	return aabb
}

func (rb *RigidBody) IsAwake() bool {
	return atomic.LoadInt32(&rb.state) == BodyStateAwake
}

func (rb *RigidBody) IsSleeping() bool {
	return atomic.LoadInt32(&rb.state) == BodyStateSleeping
}

func (rb *RigidBody) WakeUp() {
	if !rb.isStatic {
		atomic.StoreInt32(&rb.state, BodyStateAwake)
		rb.sleepTime = 0
	}
}

func (rb *RigidBody) PutToSleep() {
	if !rb.isStatic {
		atomic.StoreInt32(&rb.state, BodyStateSleeping)
		rb.velocity = Vector2D{}
		rb.angularVelocity = 0
		rb.force = Vector2D{}
		rb.torque = 0
	}
}

func (rb *RigidBody) UpdateSleepState(dt float64) {
	if rb.isStatic || rb.state == BodyStateSleeping {
		return
	}

	minMotion := SleepLinearTolerance * SleepLinearTolerance

	velSq := rb.velocity.MagnitudeSquared()
	angVelSq := rb.angularVelocity * rb.angularVelocity

	if velSq < minMotion && angVelSq < SleepAngularTolerance*SleepAngularTolerance {
		rb.sleepTime += dt
		if rb.sleepTime >= SleepTime {
			rb.PutToSleep()
		}
	} else {
		rb.sleepTime = 0
	}
}

func (rb *RigidBody) Integrate(dt float64, gravity Vector2D) {
	if rb.isStatic || rb.state == BodyStateSleeping {
		return
	}

	rb.mu.Lock()
	defer rb.mu.Unlock()

	rb.lastPosition = rb.position
	rb.lastAngle = rb.angle

	rb.force = rb.force.Add(gravity.Scale(rb.mass))

	rb.acceleration = rb.force.Scale(rb.invMass)
	rb.velocity = rb.velocity.Add(rb.acceleration.Scale(dt)).Scale(rb.damping)
	rb.position = rb.position.Add(rb.velocity.Scale(dt))

	angularAcceleration := rb.torque * rb.invInertia
	rb.angularVelocity = (rb.angularVelocity + angularAcceleration*dt) * rb.damping
	rb.angle += rb.angularVelocity * dt

	for rb.angle > math.Pi {
		rb.angle -= 2 * math.Pi
	}
	for rb.angle < -math.Pi {
		rb.angle += 2 * math.Pi
	}

	rb.force = Vector2D{}
	rb.torque = 0
}

// ==================== SPATIAL GRID ====================

type GridCell struct {
	X, Y int
}

type SpatialGrid struct {
	grid     map[GridCell][]*RigidBody
	cellSize float64
	mutex    sync.RWMutex
	bounds   AABB
}

func NewSpatialGrid(cellSize float64) *SpatialGrid {
	return &SpatialGrid{
		grid:     make(map[GridCell][]*RigidBody),
		cellSize: cellSize,
		bounds:   AABB{Min: Vector2D{X: -1000, Y: -1000}, Max: Vector2D{X: 1000, Y: 1000}},
	}
}

func (sg *SpatialGrid) Clear() {
	sg.mutex.Lock()
	for key := range sg.grid {
		sg.grid[key] = sg.grid[key][:0]
	}
	sg.mutex.Unlock()
}

func (sg *SpatialGrid) Insert(body *RigidBody) {
	if !body.IsAwake() {
		return
	}

	aabb := body.GetAABB()
	minCell := sg.getCell(aabb.Min)
	maxCell := sg.getCell(aabb.Max)

	sg.mutex.Lock()
	for x := minCell.X; x <= maxCell.X; x++ {
		for y := minCell.Y; y <= maxCell.Y; y++ {
			cell := GridCell{X: x, Y: y}
			sg.grid[cell] = append(sg.grid[cell], body)
		}
	}
	sg.mutex.Unlock()
}

func (sg *SpatialGrid) GetPotentialCollisions() [][2]*RigidBody {
	sg.mutex.RLock()
	defer sg.mutex.RUnlock()

	var pairs [][2]*RigidBody
	seen := make(map[[2]uint64]bool)

	for _, bodies := range sg.grid {
		for i := 0; i < len(bodies); i++ {
			for j := i + 1; j < len(bodies); j++ {
				bodyA, bodyB := bodies[i], bodies[j]
				if bodyA.isStatic && bodyB.isStatic {
					continue
				}

				if !bodyA.IsAwake() && !bodyB.IsAwake() {
					continue
				}

				var key [2]uint64
				if bodyA.id < bodyB.id {
					key = [2]uint64{bodyA.id, bodyB.id}
				} else {
					key = [2]uint64{bodyB.id, bodyA.id}
				}

				if !seen[key] {
					seen[key] = true
					pairs = append(pairs, [2]*RigidBody{bodyA, bodyB})
				}
			}
		}
	}

	return pairs
}

func (sg *SpatialGrid) getCell(pos Vector2D) GridCell {
	return GridCell{
		X: int(math.Floor(pos.X / sg.cellSize)),
		Y: int(math.Floor(pos.Y / sg.cellSize)),
	}
}

// ==================== COLLISION DETECTION ====================

func DetectCollision(bodyA, bodyB *RigidBody, pool *ObjectPool) *CollisionManifold {
	if !bodyA.IsAwake() && !bodyB.IsAwake() {
		return nil
	}

	aabbA := bodyA.GetAABB()
	aabbB := bodyB.GetAABB()
	if !aabbA.Overlaps(aabbB) {
		return nil
	}

	shapeTypeA := bodyA.shape.GetType()
	shapeTypeB := bodyB.shape.GetType()

	switch {
	case shapeTypeA == ShapeTypeCircle && shapeTypeB == ShapeTypeCircle:
		return detectCircleCircle(bodyA, bodyB, bodyA.shape.(*CircleShape), bodyB.shape.(*CircleShape), pool)
	case shapeTypeA == ShapeTypeBox && shapeTypeB == ShapeTypeBox:
		return detectBoxBox(bodyA, bodyB, bodyA.shape.(*BoxShape), bodyB.shape.(*BoxShape), pool)
	case shapeTypeA == ShapeTypeCircle && shapeTypeB == ShapeTypeBox:
		return detectCircleBox(bodyA, bodyB, bodyA.shape.(*CircleShape), bodyB.shape.(*BoxShape), pool)
	case shapeTypeA == ShapeTypeBox && shapeTypeB == ShapeTypeCircle:
		manifold := detectCircleBox(bodyB, bodyA, bodyB.shape.(*CircleShape), bodyA.shape.(*BoxShape), pool)
		if manifold != nil {
			manifold.BodyA, manifold.BodyB = manifold.BodyB, manifold.BodyA
			manifold.Normal = manifold.Normal.Scale(-1)
		}
		return manifold
	}

	return nil
}

func detectCircleCircle(bodyA, bodyB *RigidBody, circleA, circleB *CircleShape, pool *ObjectPool) *CollisionManifold {
	posA := bodyA.GetPosition()
	posB := bodyB.GetPosition()

	delta := posB.Sub(posA)
	distanceSquared := delta.MagnitudeSquared()
	totalRadius := circleA.Radius + circleB.Radius
	totalRadiusSquared := totalRadius * totalRadius

	if distanceSquared >= totalRadiusSquared {
		return nil
	}

	distance := math.Sqrt(distanceSquared)
	penetration := totalRadius - distance

	var normal Vector2D
	if distance > 0 {
		invDistance := 1.0 / distance
		normal = Vector2D{X: delta.X * invDistance, Y: delta.Y * invDistance}
	} else {
		normal = Vector2D{X: 1, Y: 0}
	}

	contactPoint := posA.Add(normal.Scale(circleA.Radius - penetration*0.5))

	manifold := pool.GetManifold()
	manifold.BodyA = bodyA
	manifold.BodyB = bodyB
	manifold.Normal = normal
	manifold.Penetration = penetration
	manifold.ContactPoints = append(manifold.ContactPoints, contactPoint)
	manifold.Restitution = math.Min(bodyA.restitution, bodyB.restitution)
	manifold.Friction = math.Sqrt(bodyA.friction * bodyB.friction)

	return manifold
}

func detectBoxBox(bodyA, bodyB *RigidBody, boxA, boxB *BoxShape, pool *ObjectPool) *CollisionManifold {
	aabbA := bodyA.GetAABB()
	aabbB := bodyB.GetAABB()

	overlapX := math.Min(aabbA.Max.X, aabbB.Max.X) - math.Max(aabbA.Min.X, aabbB.Min.X)
	overlapY := math.Min(aabbA.Max.Y, aabbB.Max.Y) - math.Max(aabbA.Min.Y, aabbB.Min.Y)

	if overlapX <= 0 || overlapY <= 0 {
		return nil
	}

	var normal Vector2D
	var penetration float64

	if overlapX < overlapY {
		penetration = overlapX
		if aabbA.Min.X < aabbB.Min.X {
			normal = Vector2D{X: -1, Y: 0}
		} else {
			normal = Vector2D{X: 1, Y: 0}
		}
	} else {
		penetration = overlapY
		if aabbA.Min.Y < aabbB.Min.Y {
			normal = Vector2D{X: 0, Y: -1}
		} else {
			normal = Vector2D{X: 0, Y: 1}
		}
	}

	posA := bodyA.GetPosition()
	posB := bodyB.GetPosition()
	contactPoint := posA.Add(posB).Scale(0.5)

	manifold := pool.GetManifold()
	manifold.BodyA = bodyA
	manifold.BodyB = bodyB
	manifold.Normal = normal
	manifold.Penetration = penetration
	manifold.ContactPoints = append(manifold.ContactPoints, contactPoint)
	manifold.Restitution = math.Min(bodyA.restitution, bodyB.restitution)
	manifold.Friction = math.Sqrt(bodyA.friction * bodyB.friction)

	return manifold
}

func detectCircleBox(circleBody, boxBody *RigidBody, circle *CircleShape, box *BoxShape, pool *ObjectPool) *CollisionManifold {
	circlePos := circleBody.GetPosition()
	boxPos := boxBody.GetPosition()

	halfW, halfH := box.Width*0.5, box.Height*0.5
	closest := Vector2D{
		X: math.Max(boxPos.X-halfW, math.Min(circlePos.X, boxPos.X+halfW)),
		Y: math.Max(boxPos.Y-halfH, math.Min(circlePos.Y, boxPos.Y+halfH)),
	}

	delta := circlePos.Sub(closest)
	distanceSquared := delta.MagnitudeSquared()
	radiusSquared := circle.Radius * circle.Radius

	if distanceSquared >= radiusSquared {
		return nil
	}

	distance := math.Sqrt(distanceSquared)
	penetration := circle.Radius - distance

	var normal Vector2D
	if distance > 0 {
		invDistance := 1.0 / distance
		normal = Vector2D{X: delta.X * invDistance, Y: delta.Y * invDistance}
	} else {
		xDist := math.Min(circlePos.X-(boxPos.X-halfW), (boxPos.X+halfW)-circlePos.X)
		yDist := math.Min(circlePos.Y-(boxPos.Y-halfH), (boxPos.Y+halfH)-circlePos.Y)

		if xDist < yDist {
			if circlePos.X < boxPos.X {
				normal = Vector2D{X: -1, Y: 0}
			} else {
				normal = Vector2D{X: 1, Y: 0}
			}
			penetration = xDist + circle.Radius
		} else {
			if circlePos.Y < boxPos.Y {
				normal = Vector2D{X: 0, Y: -1}
			} else {
				normal = Vector2D{X: 0, Y: 1}
			}
			penetration = yDist + circle.Radius
		}
	}

	manifold := pool.GetManifold()
	manifold.BodyA = circleBody
	manifold.BodyB = boxBody
	manifold.Normal = normal
	manifold.Penetration = penetration
	manifold.ContactPoints = append(manifold.ContactPoints, closest)
	manifold.Restitution = math.Min(circleBody.restitution, boxBody.restitution)
	manifold.Friction = math.Sqrt(circleBody.friction * boxBody.friction)

	return manifold
}

// ==================== COLLISION RESOLUTION ====================

func ResolveCollision(manifold *CollisionManifold) {
	bodyA := manifold.BodyA
	bodyB := manifold.BodyB

	if bodyA.isStatic && bodyB.isStatic {
		return
	}

	bodyA.WakeUp()
	bodyB.WakeUp()

	velA := bodyA.GetVelocity()
	velB := bodyB.GetVelocity()
	relativeVelocity := velB.Sub(velA)

	velAlongNormal := relativeVelocity.Dot(manifold.Normal)

	if velAlongNormal > 0 {
		return
	}

	e := manifold.Restitution
	j := -(1 + e) * velAlongNormal
	j /= bodyA.invMass + bodyB.invMass

	impulse := manifold.Normal.Scale(j)

	if len(manifold.ContactPoints) > 0 {
		contactPoint := manifold.ContactPoints[0]
		bodyA.ApplyImpulse(impulse.Scale(-1), contactPoint)
		bodyB.ApplyImpulse(impulse, contactPoint)
	}

	applyFriction(manifold, j)
	correctPositions(manifold)
}

func applyFriction(manifold *CollisionManifold, normalImpulse float64) {
	bodyA := manifold.BodyA
	bodyB := manifold.BodyB

	velA := bodyA.GetVelocity()
	velB := bodyB.GetVelocity()
	relativeVelocity := velB.Sub(velA)

	tangent := relativeVelocity.Sub(manifold.Normal.Scale(relativeVelocity.Dot(manifold.Normal)))
	tangentMagSq := tangent.MagnitudeSquared()

	if tangentMagSq < 1e-8 {
		return
	}

	tangent = tangent.Scale(1.0 / math.Sqrt(tangentMagSq))

	jt := -relativeVelocity.Dot(tangent)
	jt /= bodyA.invMass + bodyB.invMass

	mu := manifold.Friction
	normalForce := math.Abs(normalImpulse)

	var frictionImpulse Vector2D
	if math.Abs(jt) < normalForce*mu {
		frictionImpulse = tangent.Scale(jt)
	} else {
		dynamicFriction := mu * 0.8
		frictionImpulse = tangent.Scale(-normalForce * dynamicFriction)
	}

	if len(manifold.ContactPoints) > 0 {
		contactPoint := manifold.ContactPoints[0]
		bodyA.ApplyImpulse(frictionImpulse.Scale(-1), contactPoint)
		bodyB.ApplyImpulse(frictionImpulse, contactPoint)
	}
}

func correctPositions(manifold *CollisionManifold) {
	const percent = 0.4
	const slop = 0.005

	bodyA := manifold.BodyA
	bodyB := manifold.BodyB

	if manifold.Penetration <= slop {
		return
	}

	correction := (manifold.Penetration - slop) / (bodyA.invMass + bodyB.invMass) * percent
	correctionVector := manifold.Normal.Scale(correction)

	bodyA.mu.Lock()
	bodyB.mu.Lock()

	bodyA.position = bodyA.position.Sub(correctionVector.Scale(bodyA.invMass))
	bodyB.position = bodyB.position.Add(correctionVector.Scale(bodyB.invMass))

	bodyB.mu.Unlock()
	bodyA.mu.Unlock()
}

// ==================== WORKER POOL ====================

type Task struct {
	Execute func() error
	ID      int
}

type WorkerPool struct {
	workers    int
	taskQueue  chan TaskExecution
	wg         sync.WaitGroup
	quit       chan struct{}
	once       sync.Once
	activeJobs int64
	totalJobs  int64
}

type TaskExecution struct {
	task   Task
	result chan<- error
}

func NewWorkerPool(workers int) *WorkerPool {
	wp := &WorkerPool{
		workers:   workers,
		taskQueue: make(chan TaskExecution, workers*8),
		quit:      make(chan struct{}),
	}
	wp.start()
	return wp
}

func (wp *WorkerPool) start() {
	for i := 0; i < wp.workers; i++ {
		wp.wg.Add(1)
		go wp.worker(i)
	}
}

func (wp *WorkerPool) worker(id int) {
	defer wp.wg.Done()

	for {
		select {
		case execution := <-wp.taskQueue:
			atomic.AddInt64(&wp.activeJobs, 1)
			err := execution.task.Execute()
			atomic.AddInt64(&wp.activeJobs, -1)
			atomic.AddInt64(&wp.totalJobs, 1)

			select {
			case execution.result <- err:
			case <-wp.quit:
				return
			}
		case <-wp.quit:
			return
		}
	}
}

func (wp *WorkerPool) Submit(task Task, result chan<- error) {
	select {
	case wp.taskQueue <- TaskExecution{task: task, result: result}:
	case <-wp.quit:
		result <- fmt.Errorf("worker pool closed")
	}
}

func (wp *WorkerPool) GetStats() (active int64, total int64) {
	return atomic.LoadInt64(&wp.activeJobs), atomic.LoadInt64(&wp.totalJobs)
}

func (wp *WorkerPool) Close() {
	wp.once.Do(func() {
		close(wp.quit)
		wp.wg.Wait()
	})
}

// ==================== PHYSICS WORLD ====================

type PhysicsWorld struct {
	bodies      []*RigidBody
	gravity     Vector2D
	spatialGrid *SpatialGrid
	workerPool  *WorkerPool
	objectPool  *ObjectPool

	bodyMutex sync.RWMutex
	stepMutex sync.Mutex

	stepCounter      int64
	bodyCounter      int64
	collisionCounter int64
	sleepingBodies   int64

	maxWorkers      int
	iterations      int
	timeStep        float64
	sleepingEnabled bool

	accumulator float64
	maxStepSize float64
	minStepSize float64
	targetFPS   float64
}

func NewPhysicsWorld(gravity Vector2D, maxWorkers int) *PhysicsWorld {
	return &PhysicsWorld{
		bodies:          make([]*RigidBody, 0, 2000),
		gravity:         gravity,
		spatialGrid:     NewSpatialGrid(25.0),
		workerPool:      NewWorkerPool(maxWorkers),
		objectPool:      NewObjectPool(),
		maxWorkers:      maxWorkers,
		iterations:      6,
		timeStep:        1.0 / 60.0,
		sleepingEnabled: true,
		maxStepSize:     1.0 / 30.0,
		minStepSize:     1.0 / 120.0,
		targetFPS:       60.0,
	}
}

func (pw *PhysicsWorld) AddBody(body *RigidBody) {
	pw.bodyMutex.Lock()
	pw.bodies = append(pw.bodies, body)
	atomic.AddInt64(&pw.bodyCounter, 1)
	pw.bodyMutex.Unlock()
}

func (pw *PhysicsWorld) Step(ctx context.Context, dt float64) error {
	pw.stepMutex.Lock()
	defer pw.stepMutex.Unlock()

	atomic.AddInt64(&pw.stepCounter, 1)

	pw.spatialGrid.Clear()
	pw.bodyMutex.RLock()
	for _, body := range pw.bodies {
		if body.IsAwake() {
			pw.spatialGrid.Insert(body)
		}
	}

	bodiesSnapshot := make([]*RigidBody, len(pw.bodies))
	copy(bodiesSnapshot, pw.bodies)
	pw.bodyMutex.RUnlock()

	var sleepingCount int64
	for _, body := range bodiesSnapshot {
		if body.IsSleeping() {
			sleepingCount++
		}
	}
	atomic.StoreInt64(&pw.sleepingBodies, sleepingCount)

	tasks := pw.createPhysicsTasks(bodiesSnapshot, dt)

	if len(tasks) == 0 {
		return nil
	}

	results := make(chan error, len(tasks))
	for i, task := range tasks {
		select {
		case <-ctx.Done():
			return ctx.Err()
		default:
			task.ID = i
			pw.workerPool.Submit(task, results)
		}
	}

	for i := 0; i < len(tasks); i++ {
		select {
		case err := <-results:
			if err != nil {
				return err
			}
		case <-ctx.Done():
			return ctx.Err()
		}
	}

	if pw.sleepingEnabled {
		pw.updateSleepStates(bodiesSnapshot, dt)
	}

	return nil
}

func (pw *PhysicsWorld) createPhysicsTasks(bodies []*RigidBody, dt float64) []Task {
	if len(bodies) == 0 {
		return nil
	}

	var tasks []Task
	taskID := 0

	var awakeBodies []*RigidBody
	for _, body := range bodies {
		if body.IsAwake() {
			awakeBodies = append(awakeBodies, body)
		}
	}

	if len(awakeBodies) == 0 {
		return nil
	}

	chunkSize := max(1, len(awakeBodies)/(pw.maxWorkers*2))
	for i := 0; i < len(awakeBodies); i += chunkSize {
		end := min(i+chunkSize, len(awakeBodies))
		chunk := awakeBodies[i:end]

		tasks = append(tasks, Task{
			ID: taskID,
			Execute: func() error {
				for _, body := range chunk {
					body.Integrate(dt, pw.gravity)
				}
				return nil
			},
		})
		taskID++
	}

	collisionTask := Task{
		ID: taskID,
		Execute: func() error {
			return pw.processCollisions()
		},
	}
	tasks = append(tasks, collisionTask)

	return tasks
}

func (pw *PhysicsWorld) processCollisions() error {
	pairs := pw.spatialGrid.GetPotentialCollisions()

	if len(pairs) == 0 {
		return nil
	}

	atomic.StoreInt64(&pw.collisionCounter, int64(len(pairs)))

	batchSize := max(1, len(pairs)/(pw.maxWorkers*4))
	var wg sync.WaitGroup

	for i := 0; i < len(pairs); i += batchSize {
		end := min(i+batchSize, len(pairs))
		batch := pairs[i:end]

		wg.Add(1)
		go func(batch [][2]*RigidBody) {
			defer wg.Done()
			pw.processBatchCollisions(batch)
		}(batch)
	}

	wg.Wait()
	return nil
}

func (pw *PhysicsWorld) processBatchCollisions(batch [][2]*RigidBody) {
	manifolds := make([]*CollisionManifold, 0, len(batch))

	for _, pair := range batch {
		manifold := DetectCollision(pair[0], pair[1], pw.objectPool)
		if manifold != nil {
			manifolds = append(manifolds, manifold)
		}
	}

	for iter := 0; iter < pw.iterations; iter++ {
		for _, manifold := range manifolds {
			ResolveCollision(manifold)
		}
	}

	for _, manifold := range manifolds {
		pw.objectPool.PutManifold(manifold)
	}
}

func (pw *PhysicsWorld) updateSleepStates(bodies []*RigidBody, dt float64) {
	var wg sync.WaitGroup
	chunkSize := max(1, len(bodies)/pw.maxWorkers)

	for i := 0; i < len(bodies); i += chunkSize {
		end := min(i+chunkSize, len(bodies))
		chunk := bodies[i:end]

		wg.Add(1)
		go func(chunk []*RigidBody) {
			defer wg.Done()
			for _, body := range chunk {
				body.UpdateSleepState(dt)
			}
		}(chunk)
	}

	wg.Wait()
}

func (pw *PhysicsWorld) GetBodiesCount() int64 {
	return atomic.LoadInt64(&pw.bodyCounter)
}

func (pw *PhysicsWorld) GetStepCount() int64 {
	return atomic.LoadInt64(&pw.stepCounter)
}

func (pw *PhysicsWorld) GetCollisionCount() int64 {
	return atomic.LoadInt64(&pw.collisionCounter)
}

func (pw *PhysicsWorld) GetSleepingBodiesCount() int64 {
	return atomic.LoadInt64(&pw.sleepingBodies)
}

func (pw *PhysicsWorld) Close() {
	pw.workerPool.Close()
}

// ==================== PHYSICS ENGINE ====================

type PhysicsEngine struct {
	world     *PhysicsWorld
	running   int32
	targetFPS int
	stats     struct {
		fps           float64
		lastFrameTime time.Time
		frameCount    int64
		avgFrameTime  float64
		minFrameTime  float64
		maxFrameTime  float64
		frameTimeSum  float64
	}
	frameHistory []float64
	historySize  int
}

func NewPhysicsEngine() *PhysicsEngine {
	maxWorkers := runtime.NumCPU()
	pe := &PhysicsEngine{
		world:       NewPhysicsWorld(NewVector2D(0, -9.81), maxWorkers),
		targetFPS:   60,
		historySize: 100,
	}
	pe.frameHistory = make([]float64, 0, pe.historySize)
	return pe
}

func (pe *PhysicsEngine) AddCircle(mass float64, position Vector2D, radius float64) *RigidBody {
	shape := NewCircleShape(radius)
	body := NewRigidBody(mass, position, shape)
	pe.world.AddBody(body)
	return body
}

func (pe *PhysicsEngine) AddBox(mass float64, position Vector2D, width, height float64) *RigidBody {
	shape := NewBoxShape(width, height)
	body := NewRigidBody(mass, position, shape)
	pe.world.AddBody(body)
	return body
}

func (pe *PhysicsEngine) SetGravity(gravity Vector2D) {
	pe.world.gravity = gravity
}

func (pe *PhysicsEngine) Run(ctx context.Context) error {
	if !atomic.CompareAndSwapInt32(&pe.running, 0, 1) {
		return fmt.Errorf("engine already running")
	}
	defer atomic.StoreInt32(&pe.running, 0)

	ticker := time.NewTicker(time.Duration(1000/pe.targetFPS) * time.Millisecond)
	defer ticker.Stop()

	pe.stats.lastFrameTime = time.Now()

	for {
		select {
		case <-ticker.C:
			start := time.Now()

			if err := pe.world.Step(ctx, 1.0/float64(pe.targetFPS)); err != nil {
				return err
			}

			pe.updateStats(start)

		case <-ctx.Done():
			pe.world.Close()
			return ctx.Err()
		}
	}
}

func (pe *PhysicsEngine) updateStats(frameStart time.Time) {
	now := time.Now()
	frameTime := now.Sub(pe.stats.lastFrameTime).Seconds()
	currentFrameTime := now.Sub(frameStart).Seconds()

	pe.stats.fps = 1.0 / frameTime
	pe.stats.lastFrameTime = now
	atomic.AddInt64(&pe.stats.frameCount, 1)

	pe.stats.frameTimeSum += currentFrameTime
	frameCount := atomic.LoadInt64(&pe.stats.frameCount)
	pe.stats.avgFrameTime = pe.stats.frameTimeSum / float64(frameCount)

	if pe.stats.minFrameTime == 0 || currentFrameTime < pe.stats.minFrameTime {
		pe.stats.minFrameTime = currentFrameTime
	}
	if currentFrameTime > pe.stats.maxFrameTime {
		pe.stats.maxFrameTime = currentFrameTime
	}

	pe.frameHistory = append(pe.frameHistory, currentFrameTime)
	if len(pe.frameHistory) > pe.historySize {
		pe.frameHistory = pe.frameHistory[1:]
	}
}

func (pe *PhysicsEngine) GetStats() (fps float64, bodies int64, steps int64, frames int64) {
	return pe.stats.fps, pe.world.GetBodiesCount(), pe.world.GetStepCount(), atomic.LoadInt64(&pe.stats.frameCount)
}

func (pe *PhysicsEngine) GetAdvancedStats() (fps, avgFrameTime, minFrameTime, maxFrameTime float64, bodies, awake, collisions, workerActive, workerTotal int64) {
	workerActive, workerTotal = pe.world.workerPool.GetStats()
	return pe.stats.fps,
		pe.stats.avgFrameTime * 1000,
		pe.stats.minFrameTime * 1000,
		pe.stats.maxFrameTime * 1000,
		pe.world.GetBodiesCount(),
		pe.world.GetBodiesCount() - pe.world.GetSleepingBodiesCount(),
		pe.world.GetCollisionCount(),
		workerActive,
		workerTotal
}

// ==================== SCENE CONFIGURATION ====================

type SceneConfig struct {
	Bodies   []BodyConfig `json:"bodies"`
	Gravity  Vector2D     `json:"gravity"`
	Duration float64      `json:"duration"`
}

type BodyConfig struct {
	Type     string      `json:"type"`
	Mass     float64     `json:"mass"`
	Position Vector2D    `json:"position"`
	Velocity Vector2D    `json:"velocity"`
	Shape    ShapeConfig `json:"shape"`
}

type ShapeConfig struct {
	Radius float64 `json:"radius,omitempty"`
	Width  float64 `json:"width,omitempty"`
	Height float64 `json:"height,omitempty"`
}

func LoadSceneFromFile(filename string) (*SceneConfig, error) {
	data, err := os.ReadFile(filename)
	if err != nil {
		return nil, err
	}

	var config SceneConfig
	if err := json.Unmarshal(data, &config); err != nil {
		return nil, err
	}

	return &config, nil
}

func (pe *PhysicsEngine) LoadScene(config *SceneConfig) error {
	pe.SetGravity(config.Gravity)

	for _, bodyConfig := range config.Bodies {
		var body *RigidBody

		switch strings.ToLower(bodyConfig.Type) {
		case "circle":
			body = pe.AddCircle(bodyConfig.Mass, bodyConfig.Position, bodyConfig.Shape.Radius)
		case "box":
			body = pe.AddBox(bodyConfig.Mass, bodyConfig.Position, bodyConfig.Shape.Width, bodyConfig.Shape.Height)
		default:
			return fmt.Errorf("unknown body type: %s", bodyConfig.Type)
		}

		if bodyConfig.Velocity.X != 0 || bodyConfig.Velocity.Y != 0 {
			body.mu.Lock()
			body.velocity = bodyConfig.Velocity
			body.mu.Unlock()
		}
	}

	return nil
}

// ==================== HELPER FUNCTIONS ====================

func min(a, b int) int {
	if a < b {
		return a
	}
	return b
}

func max(a, b int) int {
	if a > b {
		return a
	}
	return b
}

// ==================== CLI CONFIGURATION ====================

type Config struct {
	// Simulation parameters
	GravityX float64
	GravityY float64
	TimeStep float64
	Duration float64
	MaxFPS   int

	// Performance settings
	Workers      int
	Iterations   int
	SleepEnabled bool

	// Output settings
	Verbose       bool
	Quiet         bool
	StatsInterval float64
	ProfileCPU    string
	ProfileMem    string

	// Scene settings
	SceneFile   string
	BodiesCount int
	SceneType   string

	// Engine settings
	Damping     float64
	Restitution float64
	Friction    float64
}

func parseFlags() *Config {
	config := &Config{}

	// Simulation parameters
	flag.Float64Var(&config.GravityX, "gravity-x", 0.0, "gravity X component")
	flag.Float64Var(&config.GravityY, "gravity-y", -9.81, "gravity Y component")
	flag.Float64Var(&config.TimeStep, "timestep", 1.0/60.0, "physics time step")
	flag.Float64Var(&config.Duration, "duration", 0, "simulation duration in seconds (0 = infinite)")
	flag.IntVar(&config.MaxFPS, "fps", 60, "maximum frames per second")

	// Performance settings
	flag.IntVar(&config.Workers, "workers", runtime.NumCPU(), "number of worker threads")
	flag.IntVar(&config.Iterations, "iterations", 6, "collision resolution iterations")
	flag.BoolVar(&config.SleepEnabled, "sleep", true, "enable body sleeping")

	// Output settings
	flag.BoolVar(&config.Verbose, "verbose", false, "verbose output")
	flag.BoolVar(&config.Quiet, "quiet", false, "minimal output")
	flag.Float64Var(&config.StatsInterval, "stats-interval", 2.0, "statistics reporting interval")
	flag.StringVar(&config.ProfileCPU, "profile-cpu", "", "CPU profile output file")
	flag.StringVar(&config.ProfileMem, "profile-mem", "", "memory profile output file")

	// Scene settings
	flag.StringVar(&config.SceneFile, "scene", "", "JSON scene file to load")
	flag.IntVar(&config.BodiesCount, "bodies", 100, "number of bodies for generated scenes")
	flag.StringVar(&config.SceneType, "scene-type", "default", "scene type (default, pyramid, rain, container)")

	// Engine settings
	flag.Float64Var(&config.Damping, "damping", 0.999, "global damping factor")
	flag.Float64Var(&config.Restitution, "restitution", 0.8, "default restitution")
	flag.Float64Var(&config.Friction, "friction", 0.3, "default friction")

	// Version flag
	var showVersion bool
	flag.BoolVar(&showVersion, "version", false, "show version information")

	// Custom usage
	flag.Usage = func() {
		fmt.Fprintf(os.Stderr, "Physics2D - High-Performance 2D Physics Engine\n\n")
		fmt.Fprintf(os.Stderr, "Usage: %s [OPTIONS]\n\n", os.Args[0])
		fmt.Fprintf(os.Stderr, "Options:\n")
		flag.PrintDefaults()
		fmt.Fprintf(os.Stderr, "\nExamples:\n")
		fmt.Fprintf(os.Stderr, "  %s -bodies 500 -scene-type pyramid\n", os.Args[0])
		fmt.Fprintf(os.Stderr, "  %s -scene scene.json -duration 10\n", os.Args[0])
		fmt.Fprintf(os.Stderr, "  %s -profile-cpu cpu.prof -verbose\n", os.Args[0])
		fmt.Fprintf(os.Stderr, "\nVersion: %s\n", Version)
	}

	flag.Parse()

	if showVersion {
		fmt.Printf("Physics2D version %s\n", Version)
		fmt.Printf("Built: %s\n", BuildTime)
		fmt.Printf("Go: %s\n", GoVersion)
		os.Exit(0)
	}

	// Validate configuration
	if err := validateConfig(config); err != nil {
		log.Fatalf("Invalid configuration: %v", err)
	}

	return config
}

func validateConfig(config *Config) error {
	if config.Workers < 1 {
		return fmt.Errorf("workers must be at least 1")
	}
	if config.MaxFPS < 1 || config.MaxFPS > 1000 {
		return fmt.Errorf("fps must be between 1 and 1000")
	}
	if config.Duration < 0 {
		return fmt.Errorf("duration cannot be negative")
	}
	if config.BodiesCount < 1 {
		return fmt.Errorf("bodies count must be at least 1")
	}
	if config.Iterations < 1 {
		return fmt.Errorf("iterations must be at least 1")
	}

	validSceneTypes := map[string]bool{
		"default": true, "pyramid": true, "rain": true,
		"container": true, "pendulum": true, "mixed": true,
	}
	if !validSceneTypes[config.SceneType] {
		return fmt.Errorf("invalid scene type: %s", config.SceneType)
	}

	return nil
}

// ==================== SCENE GENERATORS ====================

func generateScene(engine *PhysicsEngine, config *Config) {
	switch config.SceneType {
	case "pyramid":
		generatePyramidScene(engine, config.BodiesCount)
	case "rain":
		generateRainScene(engine, config.BodiesCount)
	case "container":
		generateContainerScene(engine, config.BodiesCount)
	case "pendulum":
		generatePendulumScene(engine, config.BodiesCount)
	case "mixed":
		generateMixedScene(engine, config.BodiesCount)
	default:
		generateDefaultScene(engine, config.BodiesCount)
	}
}

func generateDefaultScene(engine *PhysicsEngine, bodyCount int) {
	// Ground
	engine.AddBox(0, NewVector2D(0, -50), 200, 10)

	// Dynamic objects
	for i := 0; i < bodyCount; i++ {
		x := (rand.Float64() - 0.5) * 150
		y := rand.Float64()*50 + 50

		if rand.Float64() < 0.6 {
			radius := rand.Float64()*2 + 1
			mass := radius * radius * math.Pi
			engine.AddCircle(mass, NewVector2D(x, y), radius)
		} else {
			size := rand.Float64()*3 + 1
			mass := size * size
			engine.AddBox(mass, NewVector2D(x, y), size, size)
		}
	}
}

func generatePyramidScene(engine *PhysicsEngine, bodyCount int) {
	engine.AddBox(0, NewVector2D(0, -10), 200, 5)

	levels := int(math.Sqrt(float64(bodyCount))) + 1
	boxSize := 2.0
	y := 0.0

	for level := levels; level > 0; level-- {
		for i := 0; i < level; i++ {
			x := float64(i-level/2) * boxSize
			engine.AddBox(1.0, NewVector2D(x, y), boxSize*0.9, boxSize*0.9)
		}
		y += boxSize
	}
}

func generateRainScene(engine *PhysicsEngine, bodyCount int) {
	// Boundaries
	engine.AddBox(0, NewVector2D(0, -50), 300, 10)
	engine.AddBox(0, NewVector2D(-150, 0), 10, 100)
	engine.AddBox(0, NewVector2D(150, 0), 10, 100)

	// Falling objects
	for i := 0; i < bodyCount; i++ {
		x := (rand.Float64() - 0.5) * 250
		y := rand.Float64()*200 + 100

		if rand.Float64() < 0.7 {
			radius := rand.Float64()*2 + 0.5
			mass := radius * radius * math.Pi
			engine.AddCircle(mass, NewVector2D(x, y), radius)
		} else {
			width := rand.Float64()*3 + 1
			height := rand.Float64()*3 + 1
			mass := width * height
			engine.AddBox(mass, NewVector2D(x, y), width, height)
		}
	}
}

func generateContainerScene(engine *PhysicsEngine, bodyCount int) {
	wallThickness := 5.0
	containerWidth := 100.0
	containerHeight := 80.0

	// Container walls
	engine.AddBox(0, NewVector2D(0, -containerHeight/2), containerWidth, wallThickness)
	engine.AddBox(0, NewVector2D(-containerWidth/2, 0), wallThickness, containerHeight)
	engine.AddBox(0, NewVector2D(containerWidth/2, 0), wallThickness, containerHeight)

	// Fill container
	for i := 0; i < bodyCount; i++ {
		x := (rand.Float64() - 0.5) * (containerWidth - 20)
		y := rand.Float64()*60 + 10

		if rand.Float64() < 0.6 {
			radius := rand.Float64()*1.5 + 0.5
			mass := radius * radius * math.Pi * 0.5
			engine.AddCircle(mass, NewVector2D(x, y), radius)
		} else {
			size := rand.Float64()*2 + 1
			mass := size * size * 0.5
			engine.AddBox(mass, NewVector2D(x, y), size, size)
		}
	}
}

func generatePendulumScene(engine *PhysicsEngine, bodyCount int) {
	for i := 0; i < bodyCount/3; i++ {
		x := float64(i-bodyCount/6) * 10

		// Anchor
		engine.AddCircle(0, NewVector2D(x, 50), 0.5)

		// Bob
		bobY := 30.0
		bobMass := 2.0
		bob := engine.AddCircle(bobMass, NewVector2D(x, bobY), 1.5)
		bob.ApplyForce(NewVector2D((rand.Float64()-0.5)*100, 0))
	}
}

func generateMixedScene(engine *PhysicsEngine, bodyCount int) {
	// Complex environment with platforms
	engine.AddBox(0, NewVector2D(-75, -50), 50, 10)
	engine.AddBox(0, NewVector2D(75, -50), 50, 10)

	for i := 0; i < 5; i++ {
		x := (rand.Float64() - 0.5) * 150
		y := float64(i)*15 - 20
		width := rand.Float64()*30 + 20
		engine.AddBox(0, NewVector2D(x, y), width, 3)
	}

	// Mixed dynamic objects
	for i := 0; i < bodyCount; i++ {
		x := (rand.Float64() - 0.5) * 200
		y := rand.Float64()*100 + 50

		switch rand.Intn(3) {
		case 0:
			radius := rand.Float64()*2 + 0.5
			mass := radius * radius * math.Pi
			circle := engine.AddCircle(mass, NewVector2D(x, y), radius)
			circle.restitution = rand.Float64()*0.5 + 0.5
			circle.friction = rand.Float64()*0.5 + 0.2
		case 1:
			size := rand.Float64()*3 + 1
			mass := size * size
			box := engine.AddBox(mass, NewVector2D(x, y), size, size)
			box.restitution = rand.Float64()*0.5 + 0.3
			box.friction = rand.Float64()*0.6 + 0.3
		case 2:
			width := rand.Float64()*4 + 1
			height := rand.Float64()*2 + 0.5
			mass := width * height
			box := engine.AddBox(mass, NewVector2D(x, y), width, height)
			box.restitution = rand.Float64()*0.4 + 0.4
			box.friction = rand.Float64()*0.5 + 0.4
		}
	}
}

// ==================== MAIN APPLICATION ====================

func main() {
	// Parse command line flags
	config := parseFlags()

	// Set up logging
	if config.Quiet {
		log.SetOutput(io.Discard)
	} else if config.Verbose {
		log.SetFlags(log.LstdFlags | log.Lshortfile)
	}

	// Set up profiling
	if config.ProfileCPU != "" {
		f, err := os.Create(config.ProfileCPU)
		if err != nil {
			log.Fatal("Could not create CPU profile:", err)
		}
		defer f.Close()

		if err := pprof.StartCPUProfile(f); err != nil {
			log.Fatal("Could not start CPU profile:", err)
		}
		defer pprof.StopCPUProfile()
	}

	// Optimize for performance
	runtime.GOMAXPROCS(config.Workers)
	rand.Seed(time.Now().UnixNano())

	if !config.Quiet {
		log.Printf("Starting Physics2D Engine v%s", Version)
		log.Printf("CPU Cores: %d, Workers: %d", runtime.NumCPU(), config.Workers)
	}

	// Create physics engine
	engine := NewPhysicsEngine()
	engine.targetFPS = config.MaxFPS
	engine.world.maxWorkers = config.Workers
	engine.world.iterations = config.Iterations
	engine.world.sleepingEnabled = config.SleepEnabled

	// Set gravity
	engine.SetGravity(NewVector2D(config.GravityX, config.GravityY))

	// Load or generate scene
	if config.SceneFile != "" {
		sceneConfig, err := LoadSceneFromFile(config.SceneFile)
		if err != nil {
			log.Fatalf("Failed to load scene: %v", err)
		}

		if err := engine.LoadScene(sceneConfig); err != nil {
			log.Fatalf("Failed to setup scene: %v", err)
		}

		if sceneConfig.Duration > 0 {
			config.Duration = sceneConfig.Duration
		}

		if !config.Quiet {
			log.Printf("Loaded scene from %s", config.SceneFile)
		}
	} else {
		generateScene(engine, config)
		if !config.Quiet {
			log.Printf("Generated %s scene with %d bodies", config.SceneType, config.BodiesCount)
		}
	}

	// Create context for simulation control
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()

	// Set up duration limit
	if config.Duration > 0 {
		ctx, cancel = context.WithTimeout(ctx, time.Duration(config.Duration*float64(time.Second)))
		defer cancel()
	}

	// Handle interrupt signals
	sigChan := make(chan os.Signal, 1)
	signal.Notify(sigChan, syscall.SIGINT, syscall.SIGTERM)

	// Start statistics reporting
	if !config.Quiet {
		go reportStats(ctx, engine, config.StatsInterval, config.Verbose)
	}

	// Handle shutdown
	go func() {
		select {
		case <-sigChan:
			if !config.Quiet {
				log.Println("Shutting down gracefully...")
			}
			cancel()
		case <-ctx.Done():
			// Context finished naturally
		}
	}()

	// Run the physics simulation
	if !config.Quiet {
		log.Printf("Physics simulation started (FPS: %d, Workers: %d)", config.MaxFPS, config.Workers)
		if config.Duration > 0 {
			log.Printf("Simulation duration: %.2f seconds", config.Duration)
		} else {
			log.Println("Press Ctrl+C to stop")
		}
	}

	if err := engine.Run(ctx); err != nil && err != context.Canceled && err != context.DeadlineExceeded {
		log.Fatalf("Engine error: %v", err)
	}

	// Memory profiling
	if config.ProfileMem != "" {
		f, err := os.Create(config.ProfileMem)
		if err != nil {
			log.Printf("Could not create memory profile: %v", err)
		} else {
			defer f.Close()
			runtime.GC()
			if err := pprof.WriteHeapProfile(f); err != nil {
				log.Printf("Could not write memory profile: %v", err)
			}
		}
	}

	if !config.Quiet {
		// Final statistics
		fps, bodies, steps, frames := engine.GetStats()
		log.Printf("Simulation completed:")
		log.Printf("  Final FPS: %.1f", fps)
		log.Printf("  Bodies: %d", bodies)
		log.Printf("  Steps: %d", steps)
		log.Printf("  Frames: %d", frames)

		if steps > 0 {
			log.Printf("  Average steps/second: %.1f", float64(steps)/config.Duration)
		}
	}
}

func reportStats(ctx context.Context, engine *PhysicsEngine, interval float64, verbose bool) {
	ticker := time.NewTicker(time.Duration(interval * float64(time.Second)))
	defer ticker.Stop()

	for {
		select {
		case <-ticker.C:
			fps, avgFrame, minFrame, maxFrame, bodies, awake, collisions, workerActive, workerTotal := engine.GetAdvancedStats()

			if verbose {
				log.Printf("FPS: %.1f | Bodies: %d (Awake: %d) | Collisions: %d | "+
					"Frame: %.2f/%.2f/%.2f ms | Workers: %d/%d",
					fps, bodies, awake, collisions,
					avgFrame, minFrame, maxFrame,
					workerActive, workerTotal)
			} else {
				log.Printf("FPS: %.1f | Bodies: %d | Awake: %d | Collisions: %d",
					fps, bodies, awake, collisions)
			}

		case <-ctx.Done():
			return
		}
	}
}
