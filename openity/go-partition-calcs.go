package main

import (
	"bytes"
	"fmt"
	"text/tabwriter"
)

// to run this: go run go-partition-calcs.go

// flash in var-som-mx6
// nand: device found, Manufacturer ID: 0x2c, Chip ID: 0xdc
// nand: Micron MT29F4G08ABADAWP
// nand: 512 MiB, SLC, erase size: 128 KiB, page size: 2048, OOB size: 64
//
// original setup
// root@twc8009:~# cat /proc/mtd
// dev:    size   erasesize  name
// mtd0: 00200000 00020000 "spl"
// mtd1: 00200000 00020000 "bootloader"
// mtd2: 00800000 00020000 "kernel"
// mtd3: 1f400000 00020000 "rootfs"
//
// new:
// New Partitions
//   Device        Name      Start     Size(B)  Size(MiB)  Size(Blks)
//     mtd0         spl        0x0    0x200000          2          16
//     mtd1  bootloader   0x200000    0x200000          2          16
//     mtd2     kernel1   0x400000   0x6400000        100         800
//     mtd3     kernel2  0x6800000   0x6400000        100         800
//     mtd4        data  0xcc00000  0x13400000        308        2464
// total size = 0x20000000, 512MiB

// DTB offset in chip: 0x67e0000
// DTB offset in partition: 0x63e0000

const (
	nandBlockSize = 128 * 1024
	KiB           = 1024
	MiB           = 1024 * 1024
	GiB           = 1024 * 1024 * 1024
)

type Partition struct {
	Device string
	Name   string
	Start  uint
	Size   uint
}

func (p Partition) String() string {
	sizeMiB := float32(p.Size) / (1024 * 1024)
	sizeBlocks := float32(p.Size) / nandBlockSize
	return fmt.Sprintf("%v\t%v\t0x%x\t0x%x\t%v\t%v\t",
		p.Device, p.Name, p.Start, p.Size, sizeMiB, sizeBlocks)
}

type Partitions []Partition

func (parts Partitions) String() string {
	w := new(tabwriter.Writer)
	buf := new(bytes.Buffer)
	w.Init(buf, 5, 0, 2, ' ', tabwriter.AlignRight)

	fmt.Fprintln(w, "Device\tName\tStart\tSize(B)\tSize(MiB)\tSize(Blks)\t")

	for _, p := range parts {
		fmt.Fprintln(w, p.String())
	}

	w.Flush()

	size := parts.CalcSize()
	sizeMiB := float32(size) / (1024 * 1024)

	fmt.Fprintf(w, "total size = 0x%x, %vMiB\n", size, sizeMiB)

	return buf.String()
}

func (parts Partitions) Get(part int) Partition {
	parts_ := []Partition(parts)
	return parts_[part]
}

func (parts *Partitions) FillIn(deviceSize uint, align uint) {
	curAdr := uint(0)
	parts_ := []Partition(*parts)
	mtdDev := 0

	for i, p := range parts_ {
		// first round size to align size
		if align > 0 {
			parts_[i].Size = (p.Size / nandBlockSize) * nandBlockSize
		}

		parts_[i].Start = curAdr
		curAdr += p.Size
		parts_[i].Device = fmt.Sprintf("mtd%v", mtdDev)
		mtdDev += 1
	}

	lastI := len(parts_) - 1

	if deviceSize > 0 && parts_[lastI].Size == 0 {
		// fill in size of last partition
		parts_[lastI].Size = deviceSize - curAdr
	}
}

func (parts Partitions) CalcSize() uint {
	size := uint(0)
	parts_ := []Partition(parts)
	for _, p := range parts_ {
		size += p.Size
	}

	return size
}

func main() {
	old := Partitions{
		Partition{Name: "spl", Size: 0x200000},
		Partition{Name: "bootloader", Size: 0x200000},
		Partition{Name: "kernel", Size: 0x800000},
		Partition{Name: "rootfs", Size: 0x1f400000},
	}

	old.FillIn(0, 0)

	fmt.Printf("Old Partitions\n%v\n", old)

	new := Partitions{
		Partition{Name: "spl", Size: 0x200000},
		Partition{Name: "bootloader", Size: 0x200000},
		Partition{Name: "kernel1", Size: 100 * MiB},
		Partition{Name: "kernel2", Size: 100 * MiB},
		Partition{Name: "data"},
	}

	new.FillIn(512*MiB, 4*MiB)

	fmt.Printf("\n\nNew Partitions\n%v\n", new)

	kernel1_dtb_offset_chip := new[3].Start - nandBlockSize
	kernel1_dtb_offset_partition := new[2].Size - nandBlockSize

	kernel2_dtb_offset_chip := new[4].Start - nandBlockSize
	kernel2_dtb_offset_partition := new[3].Size - nandBlockSize

	fmt.Printf("Kernel 1 DTB offset in chip: 0x%x\n", kernel1_dtb_offset_chip)
	fmt.Printf("Kernel 1 DTB offset in partition: 0x%x\n", kernel1_dtb_offset_partition)
	fmt.Printf("Kernel 2 DTB offset in chip: 0x%x\n", kernel2_dtb_offset_chip)
	fmt.Printf("Kernel 2 DTB offset in partition: 0x%x\n", kernel2_dtb_offset_partition)
}
