using System.Windows.Input;
using WarehouseApp.Model;

namespace WarehouseApp.ViewModel
{
    public class CardViewModel : BindableObject
    {
        private int _quantity;
        public string ImageSource { get; set; }
        public string Title { get; set; }

 
        public Component Component { get; set; }

        public int Quantity
        {
            get => _quantity;
            set
            {
                _quantity = value;
                Component.Quantity = value;
                OnPropertyChanged();
            }
        }

        public ICommand IncrementCommand { get; }
        public ICommand DecrementCommand { get; }

        public CardViewModel(string title, string imageSource, Component component)
        {
            Title = title;
            ImageSource = imageSource;
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
