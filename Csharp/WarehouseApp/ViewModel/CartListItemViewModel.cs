using System.Windows.Input;
using WarehouseApp.Model;

namespace WarehouseApp.ViewModel
{
    public class CartListItemViewModel : BindableObject
    {
        private int _quantity;

        public string Name { get; set; }

        public int Quantity
        {
            get => _quantity;
            set
            {
                _quantity = value;
                OnPropertyChanged();
                Component.Quantity = value;
            }
        }

        public Component Component { get; set; }

        public ICommand IncrementCommand { get; }

        public ICommand DecrementCommand { get; }

        public CartListItemViewModel(string name, Component component)
        {
            Name = name;
            Component = component;
            Quantity = component.Quantity;

            IncrementCommand = new Command(() => Quantity++);

            DecrementCommand = new Command(() =>
            {
                if (Quantity > 0)
                    Quantity--;
            });
        }
    }
}
