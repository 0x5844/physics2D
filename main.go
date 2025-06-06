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
)

var (
	Version   = "dev"
	BuildTime = "unknown"
	GoVersion = "unknown"
)

type Vector2D struct {
	X, Y float64
}

func NewVector2D(x, y float64) Vector2D {
	return Vector2D{X: x, Y: y}
}

func (v Vector2D) Normalize() Vector2D {
	magSq := v.X*v.X + v.Y*v.Y
	if magSq == 0 {
		return Vector2D{}
	}
	invMag := float64(fastInvSqrt(float32(magSq)))
	return Vector2D{X: v.X * invMag, Y: v.Y * invMag}
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
	return math.FMA(v.X, other.X, v.Y*other.Y)
}

func (v Vector2D) Cross(other Vector2D) float64 {
	return math.FMA(v.X, other.Y, -v.Y*other.X)
}

func (v Vector2D) MagnitudeSquared() float64 {
	return v.X*v.X + v.Y*v.Y
}

func (v Vector2D) Magnitude() float64 {
	return math.Sqrt(v.MagnitudeSquared())
}

func (v Vector2D) DistanceSquared(other Vector2D) float64 {
	dx := v.X - other.X
	dy := v.Y - other.Y
	return dx*dx + dy*dy
}

func (v Vector2D) Distance(other Vector2D) float64 {
	return math.Sqrt(v.DistanceSquared(other))
}

func (v Vector2D) Rotate(angle float64) Vector2D {
	cos := math.Cos(angle)
	sin := math.Sin(angle)
	return Vector2D{
		X: v.X*cos - v.Y*sin,
		Y: v.X*sin + v.Y*cos,
	}
}

func (v Vector2D) Perp() Vector2D {
	return Vector2D{X: -v.Y, Y: v.X}
}

func fastInvSqrt(x float32) float32 {
	const threeHalfs = 1.5
	x2 := x * 0.5
	y := x
	i := math.Float32bits(y)
	i = 0x5f3759df - (i >> 1)
	y = math.Float32frombits(i)
	y = y * (threeHalfs - (x2 * y * y))
	return y
}

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

type Shape interface {
	GetAABB(position Vector2D, angle float64) AABB
	GetMomentOfInertia(mass float64) float64
	TestPoint(position Vector2D, angle float64, point Vector2D) bool
	GetType() ShapeType
	GetVertices(position Vector2D, angle float64) []Vector2D
	GetNormals() []Vector2D
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

func (c *CircleShape) GetAABB(position Vector2D, angle float64) AABB {
	return AABB{
		Min: Vector2D{X: position.X - c.Radius, Y: position.Y - c.Radius},
		Max: Vector2D{X: position.X + c.Radius, Y: position.Y + c.Radius},
	}
}

func (c *CircleShape) GetMomentOfInertia(mass float64) float64 {
	return 0.5 * mass * c.Radius * c.Radius
}

func (c *CircleShape) TestPoint(position Vector2D, angle float64, point Vector2D) bool {
	return position.DistanceSquared(point) <= c.Radius*c.Radius
}

func (c *CircleShape) GetType() ShapeType {
	return ShapeTypeCircle
}

func (c *CircleShape) GetVertices(position Vector2D, angle float64) []Vector2D {
	return []Vector2D{position}
}

func (c *CircleShape) GetNormals() []Vector2D {
	return []Vector2D{}
}

type BoxShape struct {
	Width, Height float64
	vertices      []Vector2D
	normals       []Vector2D
}

func NewBoxShape(width, height float64) *BoxShape {
	halfW, halfH := width*0.5, height*0.5
	vertices := []Vector2D{
		{X: -halfW, Y: -halfH},
		{X: halfW, Y: -halfH},
		{X: halfW, Y: halfH},
		{X: -halfW, Y: halfH},
	}
	normals := []Vector2D{
		{X: 0, Y: -1},
		{X: 1, Y: 0},
		{X: 0, Y: 1},
		{X: -1, Y: 0},
	}
	return &BoxShape{
		Width:    width,
		Height:   height,
		vertices: vertices,
		normals:  normals,
	}
}

func (b *BoxShape) GetAABB(position Vector2D, angle float64) AABB {
	vertices := b.GetVertices(position, angle)
	minX, maxX := vertices[0].X, vertices[0].X
	minY, maxY := vertices[0].Y, vertices[0].Y

	for _, v := range vertices[1:] {
		if v.X < minX {
			minX = v.X
		}
		if v.X > maxX {
			maxX = v.X
		}
		if v.Y < minY {
			minY = v.Y
		}
		if v.Y > maxY {
			maxY = v.Y
		}
	}

	return AABB{Min: Vector2D{X: minX, Y: minY}, Max: Vector2D{X: maxX, Y: maxY}}
}

func (b *BoxShape) GetMomentOfInertia(mass float64) float64 {
	return mass * (b.Width*b.Width + b.Height*b.Height) / 12.0
}

func (b *BoxShape) TestPoint(position Vector2D, angle float64, point Vector2D) bool {
	vertices := b.GetVertices(position, angle)
	return isPointInPolygon(point, vertices)
}

func (b *BoxShape) GetType() ShapeType {
	return ShapeTypeBox
}

func (b *BoxShape) GetVertices(position Vector2D, angle float64) []Vector2D {
	vertices := make([]Vector2D, len(b.vertices))
	for i, v := range b.vertices {
		rotated := v.Rotate(angle)
		vertices[i] = position.Add(rotated)
	}
	return vertices
}

func (b *BoxShape) GetNormals() []Vector2D {
	return b.normals
}

type PolygonShape struct {
	vertices []Vector2D
	normals  []Vector2D
	centroid Vector2D
}

func NewPolygonShape(vertices []Vector2D) *PolygonShape {
	if len(vertices) < 3 {
		panic("polygon must have at least 3 vertices")
	}

	normalizedVertices := make([]Vector2D, len(vertices))
	centroid := calculateCentroid(vertices)

	for i, v := range vertices {
		normalizedVertices[i] = v.Sub(centroid)
	}

	normals := calculateNormals(normalizedVertices)

	return &PolygonShape{
		vertices: normalizedVertices,
		normals:  normals,
		centroid: centroid,
	}
}

func calculateCentroid(vertices []Vector2D) Vector2D {
	var centroid Vector2D
	area := 0.0

	for i := 0; i < len(vertices); i++ {
		j := (i + 1) % len(vertices)
		cross := vertices[i].Cross(vertices[j])
		area += cross
		centroid = centroid.Add(vertices[i].Add(vertices[j]).Scale(cross))
	}

	area *= 0.5
	if math.Abs(area) < 1e-10 {
		sum := Vector2D{}
		for _, v := range vertices {
			sum = sum.Add(v)
		}
		return sum.Scale(1.0 / float64(len(vertices)))
	}

	return centroid.Scale(1.0 / (6.0 * area))
}

func calculateNormals(vertices []Vector2D) []Vector2D {
	normals := make([]Vector2D, len(vertices))
	for i := 0; i < len(vertices); i++ {
		j := (i + 1) % len(vertices)
		edge := vertices[j].Sub(vertices[i])
		normals[i] = edge.Perp().Normalize()
	}
	return normals
}

func (p *PolygonShape) GetAABB(position Vector2D, angle float64) AABB {
	vertices := p.GetVertices(position, angle)
	minX, maxX := vertices[0].X, vertices[0].X
	minY, maxY := vertices[0].Y, vertices[0].Y

	for _, v := range vertices[1:] {
		if v.X < minX {
			minX = v.X
		}
		if v.X > maxX {
			maxX = v.X
		}
		if v.Y < minY {
			minY = v.Y
		}
		if v.Y > maxY {
			maxY = v.Y
		}
	}

	return AABB{Min: Vector2D{X: minX, Y: minY}, Max: Vector2D{X: maxX, Y: maxY}}
}

func (p *PolygonShape) GetMomentOfInertia(mass float64) float64 {
	numerator := 0.0
	denominator := 0.0

	for i := 0; i < len(p.vertices); i++ {
		j := (i + 1) % len(p.vertices)
		v1, v2 := p.vertices[i], p.vertices[j]

		cross := math.Abs(v1.Cross(v2))
		numerator += cross * (v1.Dot(v1) + v1.Dot(v2) + v2.Dot(v2))
		denominator += cross
	}

	if denominator == 0 {
		return 0
	}

	return mass * numerator / (6.0 * denominator)
}

func (p *PolygonShape) TestPoint(position Vector2D, angle float64, point Vector2D) bool {
	vertices := p.GetVertices(position, angle)
	return isPointInPolygon(point, vertices)
}

func (p *PolygonShape) GetType() ShapeType {
	return ShapeTypePolygon
}

func (p *PolygonShape) GetVertices(position Vector2D, angle float64) []Vector2D {
	vertices := make([]Vector2D, len(p.vertices))
	for i, v := range p.vertices {
		rotated := v.Rotate(angle)
		vertices[i] = position.Add(rotated)
	}
	return vertices
}

func (p *PolygonShape) GetNormals() []Vector2D {
	return p.normals
}

func isPointInPolygon(point Vector2D, vertices []Vector2D) bool {
	inside := false
	j := len(vertices) - 1

	for i := 0; i < len(vertices); i++ {
		xi, yi := vertices[i].X, vertices[i].Y
		xj, yj := vertices[j].X, vertices[j].Y

		if ((yi > point.Y) != (yj > point.Y)) &&
			(point.X < (xj-xi)*(point.Y-yi)/(yj-yi)+xi) {
			inside = !inside
		}
		j = i
	}

	return inside
}

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

func (rb *RigidBody) GetAngle() float64 {
	rb.mu.RLock()
	angle := rb.angle
	rb.mu.RUnlock()
	return angle
}

func (rb *RigidBody) GetAABB() AABB {
	rb.mu.RLock()
	aabb := rb.shape.GetAABB(rb.position, rb.angle)
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

var collisionPairsPool = sync.Pool{
	New: func() interface{} {
		return make([][2]*RigidBody, 0, 256)
	},
}

func DetectCollision(bodyA, bodyB *RigidBody, pool *ObjectPool) *CollisionManifold {
	if (bodyA.isStatic && bodyB.isStatic) || (bodyA.IsSleeping() && bodyB.IsSleeping()) {
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
		return detectPolygonPolygon(bodyA, bodyB, pool)
	case shapeTypeA == ShapeTypePolygon && shapeTypeB == ShapeTypePolygon:
		return detectPolygonPolygon(bodyA, bodyB, pool)
	case shapeTypeA == ShapeTypeBox && shapeTypeB == ShapeTypePolygon:
		return detectPolygonPolygon(bodyA, bodyB, pool)
	case shapeTypeA == ShapeTypePolygon && shapeTypeB == ShapeTypeBox:
		return detectPolygonPolygon(bodyA, bodyB, pool)
	case shapeTypeA == ShapeTypeCircle && shapeTypeB == ShapeTypeBox:
		return detectCirclePolygon(bodyA, bodyB, pool)
	case shapeTypeA == ShapeTypeBox && shapeTypeB == ShapeTypeCircle:
		manifold := detectCirclePolygon(bodyB, bodyA, pool)
		if manifold != nil {
			manifold.BodyA, manifold.BodyB = manifold.BodyB, manifold.BodyA
			manifold.Normal = manifold.Normal.Scale(-1)
		}
		return manifold
	case shapeTypeA == ShapeTypeCircle && shapeTypeB == ShapeTypePolygon:
		return detectCirclePolygon(bodyA, bodyB, pool)
	case shapeTypeA == ShapeTypePolygon && shapeTypeB == ShapeTypeCircle:
		manifold := detectCirclePolygon(bodyB, bodyA, pool)
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

func detectPolygonPolygon(bodyA, bodyB *RigidBody, pool *ObjectPool) *CollisionManifold {
	verticesA := bodyA.shape.GetVertices(bodyA.GetPosition(), bodyA.GetAngle())
	verticesB := bodyB.shape.GetVertices(bodyB.GetPosition(), bodyB.GetAngle())

	var minPenetration float64 = math.MaxFloat64
	var separationNormal Vector2D
	var contactPoints []Vector2D

	if !checkSeparation(verticesA, verticesB, &minPenetration, &separationNormal) {
		return nil
	}

	if !checkSeparation(verticesB, verticesA, &minPenetration, &separationNormal) {
		return nil
	}

	contactPoints = findContactPoints(verticesA, verticesB, separationNormal)

	if len(contactPoints) == 0 {
		return nil
	}

	manifold := pool.GetManifold()
	manifold.BodyA = bodyA
	manifold.BodyB = bodyB
	manifold.Normal = separationNormal
	manifold.Penetration = minPenetration
	manifold.ContactPoints = contactPoints
	manifold.Restitution = math.Min(bodyA.restitution, bodyB.restitution)
	manifold.Friction = math.Sqrt(bodyA.friction * bodyB.friction)

	return manifold
}

func checkSeparation(verticesA, verticesB []Vector2D, minPenetration *float64, separationNormal *Vector2D) bool {
	for i := 0; i < len(verticesA); i++ {
		j := (i + 1) % len(verticesA)
		edge := verticesA[j].Sub(verticesA[i])
		normal := edge.Perp().Normalize()

		minA, maxA := projectVertices(verticesA, normal)
		minB, maxB := projectVertices(verticesB, normal)

		if maxA <= minB || maxB <= minA {
			return false
		}

		penetration := math.Min(maxA-minB, maxB-minA)
		if penetration < *minPenetration {
			*minPenetration = penetration
			*separationNormal = normal
		}
	}
	return true
}

func projectVertices(vertices []Vector2D, axis Vector2D) (float64, float64) {
	min := vertices[0].Dot(axis)
	max := min

	for i := 1; i < len(vertices); i++ {
		projection := vertices[i].Dot(axis)
		if projection < min {
			min = projection
		}
		if projection > max {
			max = projection
		}
	}

	return min, max
}

func findContactPoints(verticesA, verticesB []Vector2D, normal Vector2D) []Vector2D {
	var contacts []Vector2D

	for _, vertex := range verticesA {
		if isVertexInside(vertex, verticesB) {
			contacts = append(contacts, vertex)
		}
	}

	for _, vertex := range verticesB {
		if isVertexInside(vertex, verticesA) {
			contacts = append(contacts, vertex)
		}
	}

	if len(contacts) > 2 {
		contacts = contacts[:2]
	}

	return contacts
}

func isVertexInside(vertex Vector2D, polygon []Vector2D) bool {
	return isPointInPolygon(vertex, polygon)
}

func detectCirclePolygon(circleBody, polygonBody *RigidBody, pool *ObjectPool) *CollisionManifold {
	circlePos := circleBody.GetPosition()
	circle := circleBody.shape.(*CircleShape)
	polygonVertices := polygonBody.shape.GetVertices(polygonBody.GetPosition(), polygonBody.GetAngle())

	var closestPoint Vector2D
	var minDistanceSquared float64 = math.MaxFloat64

	for i := 0; i < len(polygonVertices); i++ {
		j := (i + 1) % len(polygonVertices)
		closest := closestPointOnSegment(circlePos, polygonVertices[i], polygonVertices[j])
		distSq := circlePos.DistanceSquared(closest)

		if distSq < minDistanceSquared {
			minDistanceSquared = distSq
			closestPoint = closest
		}
	}

	distance := math.Sqrt(minDistanceSquared)
	if distance >= circle.Radius {
		return nil
	}

	penetration := circle.Radius - distance
	var normal Vector2D

	if distance > 1e-8 {
		normal = circlePos.Sub(closestPoint).Normalize()
	} else {
		normal = Vector2D{X: 1, Y: 0}
	}

	manifold := pool.GetManifold()
	manifold.BodyA = circleBody
	manifold.BodyB = polygonBody
	manifold.Normal = normal
	manifold.Penetration = penetration
	manifold.ContactPoints = append(manifold.ContactPoints, closestPoint)
	manifold.Restitution = math.Min(circleBody.restitution, polygonBody.restitution)
	manifold.Friction = math.Sqrt(circleBody.friction * polygonBody.friction)

	return manifold
}

func closestPointOnSegment(point, a, b Vector2D) Vector2D {
	ab := b.Sub(a)
	ap := point.Sub(a)

	abSquared := ab.Dot(ab)
	if abSquared == 0 {
		return a
	}

	t := ap.Dot(ab) / abSquared

	if t < 0 {
		return a
	}
	if t > 1 {
		return b
	}

	return a.Add(ab.Scale(t))
}

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

type Task struct {
	Execute func() error
	ID      int
}

var taskPool = sync.Pool{
	New: func() interface{} {
		return &Task{}
	},
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

	manifoldsPool := sync.Pool{
		New: func() interface{} {
			return make([]*CollisionManifold, 0, batchSize)
		},
	}

	for i := 0; i < len(pairs); i += batchSize {
		end := min(i+batchSize, len(pairs))
		batch := pairs[i:end]

		wg.Add(1)
		go func(batch [][2]*RigidBody) {
			defer wg.Done()
			manifolds := manifoldsPool.Get().([]*CollisionManifold)[:0]
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
			manifoldsPool.Put(manifolds)
		}(batch)
	}

	wg.Wait()
	return nil
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

func (pe *PhysicsEngine) AddPolygon(mass float64, position Vector2D, vertices []Vector2D) *RigidBody {
	shape := NewPolygonShape(vertices)
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
	Radius   float64    `json:"radius,omitempty"`
	Width    float64    `json:"width,omitempty"`
	Height   float64    `json:"height,omitempty"`
	Vertices []Vector2D `json:"vertices,omitempty"`
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
		case "polygon":
			if len(bodyConfig.Shape.Vertices) < 3 {
				return fmt.Errorf("polygon must have at least 3 vertices")
			}
			body = pe.AddPolygon(bodyConfig.Mass, bodyConfig.Position, bodyConfig.Shape.Vertices)
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

type Config struct {
	GravityX      float64
	GravityY      float64
	TimeStep      float64
	Duration      float64
	MaxFPS        int
	Workers       int
	Iterations    int
	SleepEnabled  bool
	Verbose       bool
	Quiet         bool
	StatsInterval float64
	ProfileCPU    string
	ProfileMem    string
	SceneFile     string
	BodiesCount   int
	SceneType     string
	Damping       float64
	Restitution   float64
	Friction      float64
}

func parseFlags() *Config {
	config := &Config{}

	flag.Float64Var(&config.GravityX, "gravity-x", 0.0, "gravity X component")
	flag.Float64Var(&config.GravityY, "gravity-y", -9.81, "gravity Y component")
	flag.Float64Var(&config.TimeStep, "timestep", 1.0/60.0, "physics time step")
	flag.Float64Var(&config.Duration, "duration", 0, "simulation duration in seconds (0 = infinite)")
	flag.IntVar(&config.MaxFPS, "fps", 60, "maximum frames per second")
	flag.IntVar(&config.Workers, "workers", runtime.NumCPU(), "number of worker threads")
	flag.IntVar(&config.Iterations, "iterations", 6, "collision resolution iterations")
	flag.BoolVar(&config.SleepEnabled, "sleep", true, "enable body sleeping")
	flag.BoolVar(&config.Verbose, "verbose", false, "verbose output")
	flag.BoolVar(&config.Quiet, "quiet", false, "minimal output")
	flag.Float64Var(&config.StatsInterval, "stats-interval", 2.0, "statistics reporting interval")
	flag.StringVar(&config.ProfileCPU, "profile-cpu", "", "CPU profile output file")
	flag.StringVar(&config.ProfileMem, "profile-mem", "", "memory profile output file")
	flag.StringVar(&config.SceneFile, "scene", "", "JSON scene file to load")
	flag.IntVar(&config.BodiesCount, "bodies", 100, "number of bodies for generated scenes")
	flag.StringVar(&config.SceneType, "scene-type", "default", "scene type (default, pyramid, rain, container, polygon)")
	flag.Float64Var(&config.Damping, "damping", 0.999, "global damping factor")
	flag.Float64Var(&config.Restitution, "restitution", 0.8, "default restitution")
	flag.Float64Var(&config.Friction, "friction", 0.3, "default friction")

	var showVersion bool
	flag.BoolVar(&showVersion, "version", false, "show version information")

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
		"container": true, "pendulum": true, "mixed": true, "polygon": true,
	}
	if !validSceneTypes[config.SceneType] {
		return fmt.Errorf("invalid scene type: %s", config.SceneType)
	}

	return nil
}

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
	case "polygon":
		generatePolygonScene(engine, config.BodiesCount)
	default:
		generateDefaultScene(engine, config.BodiesCount)
	}
}

func generateDefaultScene(engine *PhysicsEngine, bodyCount int) {
	engine.AddBox(0, NewVector2D(0, -50), 200, 10)

	for i := 0; i < bodyCount; i++ {
		x := (rand.Float64() - 0.5) * 150
		y := rand.Float64()*50 + 50

		shapeType := rand.Intn(3)
		switch shapeType {
		case 0:
			radius := rand.Float64()*2 + 1
			mass := radius * radius * math.Pi
			engine.AddCircle(mass, NewVector2D(x, y), radius)
		case 1:
			size := rand.Float64()*3 + 1
			mass := size * size
			engine.AddBox(mass, NewVector2D(x, y), size, size)
		case 2:
			vertices := generateRandomPolygon(rand.Intn(4)+3, rand.Float64()*2+1)
			mass := calculatePolygonArea(vertices)
			engine.AddPolygon(mass, NewVector2D(x, y), vertices)
		}
	}
}

func generatePolygonScene(engine *PhysicsEngine, bodyCount int) {
	engine.AddBox(0, NewVector2D(0, -50), 300, 10)

	for i := 0; i < bodyCount; i++ {
		x := (rand.Float64() - 0.5) * 200
		y := rand.Float64()*80 + 30

		sides := rand.Intn(5) + 3
		radius := rand.Float64()*3 + 1
		vertices := generateRegularPolygon(sides, radius)

		mass := calculatePolygonArea(vertices) * 0.5
		polygon := engine.AddPolygon(mass, NewVector2D(x, y), vertices)
		polygon.restitution = rand.Float64()*0.5 + 0.3
		polygon.friction = rand.Float64()*0.4 + 0.2
	}
}

func generateRegularPolygon(sides int, radius float64) []Vector2D {
	vertices := make([]Vector2D, sides)
	angleStep := 2 * math.Pi / float64(sides)

	for i := 0; i < sides; i++ {
		angle := float64(i) * angleStep
		vertices[i] = Vector2D{
			X: radius * math.Cos(angle),
			Y: radius * math.Sin(angle),
		}
	}

	return vertices
}

func generateRandomPolygon(sides int, maxRadius float64) []Vector2D {
	vertices := make([]Vector2D, sides)
	angleStep := 2 * math.Pi / float64(sides)

	for i := 0; i < sides; i++ {
		angle := float64(i) * angleStep
		radius := rand.Float64()*maxRadius + 0.5
		vertices[i] = Vector2D{
			X: radius * math.Cos(angle),
			Y: radius * math.Sin(angle),
		}
	}

	return vertices
}

func calculatePolygonArea(vertices []Vector2D) float64 {
	area := 0.0
	for i := 0; i < len(vertices); i++ {
		j := (i + 1) % len(vertices)
		area += vertices[i].Cross(vertices[j])
	}
	return math.Abs(area) * 0.5
}

func generatePyramidScene(engine *PhysicsEngine, bodyCount int) {
	engine.AddBox(0, NewVector2D(0, -10), 200, 5)

	levels := int(math.Sqrt(float64(bodyCount))) + 1
	boxSize := 2.0
	y := 0.0

	for level := levels; level > 0; level-- {
		for i := 0; i < level; i++ {
			x := float64(i-level/2) * boxSize
			if rand.Float64() < 0.5 {
				engine.AddBox(1.0, NewVector2D(x, y), boxSize*0.9, boxSize*0.9)
			} else {
				vertices := generateRegularPolygon(rand.Intn(3)+4, boxSize*0.4)
				engine.AddPolygon(1.0, NewVector2D(x, y), vertices)
			}
		}
		y += boxSize
	}
}

func generateRainScene(engine *PhysicsEngine, bodyCount int) {
	engine.AddBox(0, NewVector2D(0, -50), 300, 10)
	engine.AddBox(0, NewVector2D(-150, 0), 10, 100)
	engine.AddBox(0, NewVector2D(150, 0), 10, 100)

	for i := 0; i < bodyCount; i++ {
		x := (rand.Float64() - 0.5) * 250
		y := rand.Float64()*200 + 100

		shapeType := rand.Intn(3)
		switch shapeType {
		case 0:
			radius := rand.Float64()*2 + 0.5
			mass := radius * radius * math.Pi
			engine.AddCircle(mass, NewVector2D(x, y), radius)
		case 1:
			width := rand.Float64()*3 + 1
			height := rand.Float64()*3 + 1
			mass := width * height
			engine.AddBox(mass, NewVector2D(x, y), width, height)
		case 2:
			vertices := generateRandomPolygon(rand.Intn(4)+3, rand.Float64()*2+0.5)
			mass := calculatePolygonArea(vertices)
			engine.AddPolygon(mass, NewVector2D(x, y), vertices)
		}
	}
}

func generateContainerScene(engine *PhysicsEngine, bodyCount int) {
	wallThickness := 5.0
	containerWidth := 100.0
	containerHeight := 80.0

	engine.AddBox(0, NewVector2D(0, -containerHeight/2), containerWidth, wallThickness)
	engine.AddBox(0, NewVector2D(-containerWidth/2, 0), wallThickness, containerHeight)
	engine.AddBox(0, NewVector2D(containerWidth/2, 0), wallThickness, containerHeight)

	for i := 0; i < bodyCount; i++ {
		x := (rand.Float64() - 0.5) * (containerWidth - 20)
		y := rand.Float64()*60 + 10

		shapeType := rand.Intn(3)
		switch shapeType {
		case 0:
			radius := rand.Float64()*1.5 + 0.5
			mass := radius * radius * math.Pi * 0.5
			engine.AddCircle(mass, NewVector2D(x, y), radius)
		case 1:
			size := rand.Float64()*2 + 1
			mass := size * size * 0.5
			engine.AddBox(mass, NewVector2D(x, y), size, size)
		case 2:
			vertices := generateRegularPolygon(rand.Intn(4)+3, rand.Float64()*1.5+0.5)
			mass := calculatePolygonArea(vertices) * 0.5
			engine.AddPolygon(mass, NewVector2D(x, y), vertices)
		}
	}
}

func generatePendulumScene(engine *PhysicsEngine, bodyCount int) {
	for i := 0; i < bodyCount/3; i++ {
		x := float64(i-bodyCount/6) * 10

		engine.AddCircle(0, NewVector2D(x, 50), 0.5)

		bobY := 30.0
		bobMass := 2.0

		if rand.Float64() < 0.5 {
			bob := engine.AddCircle(bobMass, NewVector2D(x, bobY), 1.5)
			bob.ApplyForce(NewVector2D((rand.Float64()-0.5)*100, 0))
		} else {
			vertices := generateRegularPolygon(rand.Intn(4)+3, 1.2)
			bob := engine.AddPolygon(bobMass, NewVector2D(x, bobY), vertices)
			bob.ApplyForce(NewVector2D((rand.Float64()-0.5)*100, 0))
		}
	}
}

func generateMixedScene(engine *PhysicsEngine, bodyCount int) {
	engine.AddBox(0, NewVector2D(-75, -50), 50, 10)
	engine.AddBox(0, NewVector2D(75, -50), 50, 10)

	for i := 0; i < 5; i++ {
		x := (rand.Float64() - 0.5) * 150
		y := float64(i)*15 - 20
		width := rand.Float64()*30 + 20

		if rand.Float64() < 0.7 {
			engine.AddBox(0, NewVector2D(x, y), width, 3)
		} else {
			vertices := []Vector2D{
				{X: -width / 2, Y: -1.5},
				{X: width / 2, Y: -1.5},
				{X: width / 2, Y: 1.5},
				{X: -width / 2, Y: 1.5},
			}
			engine.AddPolygon(0, NewVector2D(x, y), vertices)
		}
	}

	for i := 0; i < bodyCount; i++ {
		x := (rand.Float64() - 0.5) * 200
		y := rand.Float64()*100 + 50

		shapeType := rand.Intn(4)
		switch shapeType {
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
		case 3:
			vertices := generateRandomPolygon(rand.Intn(4)+3, rand.Float64()*2+0.5)
			mass := calculatePolygonArea(vertices)
			polygon := engine.AddPolygon(mass, NewVector2D(x, y), vertices)
			polygon.restitution = rand.Float64()*0.4 + 0.4
			polygon.friction = rand.Float64()*0.5 + 0.3
		}
	}
}

func main() {
	config := parseFlags()

	if config.Quiet {
		log.SetOutput(io.Discard)
	} else if config.Verbose {
		log.SetFlags(log.LstdFlags | log.Lshortfile)
	}

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

	runtime.GOMAXPROCS(config.Workers)
	rand.Seed(time.Now().UnixNano())

	if !config.Quiet {
		log.Printf("Starting Physics2D Engine v%s", Version)
		log.Printf("CPU Cores: %d, Workers: %d", runtime.NumCPU(), config.Workers)
	}

	engine := NewPhysicsEngine()
	engine.targetFPS = config.MaxFPS
	engine.world.maxWorkers = config.Workers
	engine.world.iterations = config.Iterations
	engine.world.sleepingEnabled = config.SleepEnabled

	engine.SetGravity(NewVector2D(config.GravityX, config.GravityY))

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

	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()

	if config.Duration > 0 {
		ctx, cancel = context.WithTimeout(ctx, time.Duration(config.Duration*float64(time.Second)))
		defer cancel()
	}

	sigChan := make(chan os.Signal, 1)
	signal.Notify(sigChan, syscall.SIGINT, syscall.SIGTERM)

	if !config.Quiet {
		go reportStats(ctx, engine, config.StatsInterval, config.Verbose)
	}

	go func() {
		select {
		case <-sigChan:
			if !config.Quiet {
				log.Println("Shutting down gracefully...")
			}
			cancel()
		case <-ctx.Done():
		}
	}()

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

func BenchmarkPhysicsEngine(engine *PhysicsEngine, steps int) (avgStepTime float64) {
	start := time.Now()
	ctx := context.Background()
	for i := 0; i < steps; i++ {
		engine.world.Step(ctx, 1.0/float64(engine.targetFPS))
	}
	dur := time.Since(start).Seconds()
	return dur / float64(steps)
}
